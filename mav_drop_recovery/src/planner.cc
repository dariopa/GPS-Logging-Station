#include <mav_drop_recovery/planner.h>

TrajectoryPlanner::TrajectoryPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
    nh_(nh),
    nh_private_(nh_private),
    current_position_(Eigen::Affine3d::Identity()) {

  // create publisher for RVIZ markers
  pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
  pub_trajectory_ = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);

  // subscriber for pose
  sub_pose_ = nh.subscribe("uav_pose", 1, &TrajectoryPlanner::uavPoseCallback, this);
  
  // service server
  takeoff_service_ = nh.advertiseService("takeoff", &TrajectoryPlanner::takeoffCallback, this);
  traverse_service_ = nh.advertiseService("traverse", &TrajectoryPlanner::traverseCallback, this);
  release_service_ = nh.advertiseService("release", &TrajectoryPlanner::releaseCallback, this);
  homecoming_service_ = nh.advertiseService("homecoming", &TrajectoryPlanner::homecomingCallback, this);

  // service call execution
  execute_service_ = nh.advertiseService("execute", &TrajectoryPlanner::executeCachedTrajectory, this);
}

void TrajectoryPlanner::loadParameters() {
  CHECK(nh_private_.getParam("wp1_z", wp1_z_) &&
        nh_private_.getParam("wp2_x", wp2_x_) && 
        nh_private_.getParam("wp2_y", wp2_y_) && 
        nh_private_.getParam("wp3_z", wp3_z_) &&
        nh_private_.getParam("v_max", v_max_) &&
        nh_private_.getParam("a_max", a_max_) &&
        nh_private_.getParam("safety_altitude", safety_altitude_))
        << "Error loading parameters!";
}

void TrajectoryPlanner::uavPoseCallback(const geometry_msgs::Pose::ConstPtr& pose) {
  tf::poseMsgToEigen(*pose, current_position_);
}

void TrajectoryPlanner::getFirstPose() {
  sp_x_ = current_position_.translation().x();
  sp_y_ = current_position_.translation().y();
  sp_z_ = current_position_.translation().z();
}

bool TrajectoryPlanner::takeoffCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {

  // check if actually a take off
  if (wp1_z_ < 0) {
    ROS_WARN("Not a takeoff - not executing!");
    return false;
  }

  // check if takeoff altitude is above safety altitude
  if (wp1_z_ < safety_altitude_) {
    ROS_WARN("Take off too low. Increase takeoff altitude - not executing!");
    return false;
  }

  mav_trajectory_generation::Vertex::Vector vertices;
  const int dimension = 3;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
  // we have 2 vertices: start = current position || end = Final point.
  mav_trajectory_generation::Vertex start(dimension), end(dimension);

  // set start point constraints
  // (current position, and everything else zero)
  start.makeStartOrEnd(current_position_.translation(), derivative_to_optimize);
  vertices.push_back(start);

  // plan final point if needed (to end position at rest).
  Eigen::Vector3d end_point_position = current_position_.translation();
  end_point_position.x() = current_position_.translation().x();
  end_point_position.y() = current_position_.translation().y();
  end_point_position.z() = current_position_.translation().z() + wp1_z_;
  end.makeStartOrEnd(end_point_position, derivative_to_optimize);
  vertices.push_back(end);

  // compute segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, v_max_, a_max_);


  // compute segment times at mach slower speed / acceleration and use for first segment.
  std::vector<double> segment_times_slow;
  segment_times_slow = estimateSegmentTimes(vertices, v_max_ * 0.4, a_max_ * 0.4);
  segment_times[0] = segment_times_slow[0];


  // solve trajectory
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.solveLinear();

  // get trajectory
  trajectory_.clear();
  opt.getTrajectory(&trajectory_);
  
  visualizeCachedTrajectory();
  return true;
}

bool TrajectoryPlanner::traverseCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {

  mav_trajectory_generation::Vertex::Vector vertices;
  const int dimension = 3;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
  // we have 2 vertices: start = current position || end = Final point.
  mav_trajectory_generation::Vertex start(dimension), end(dimension);

  // set start point constraints
  // (current position, and everything else zero)
  start.makeStartOrEnd(current_position_.translation(), derivative_to_optimize);
  vertices.push_back(start);

  // plan final point if needed (to end position at rest).
  Eigen::Vector3d end_point_position = current_position_.translation();
  end_point_position.x() = wp2_x_;
  end_point_position.y() = wp2_y_;
  end_point_position.z() = current_position_.translation().z();
  end.makeStartOrEnd(end_point_position, derivative_to_optimize);
  vertices.push_back(end);

  // compute segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, v_max_, a_max_);

  // solve trajectory
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.solveLinear();

  // get trajectory
  trajectory_.clear();
  opt.getTrajectory(&trajectory_);
  
  visualizeCachedTrajectory();
  return true;
}

bool TrajectoryPlanner::releaseCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {

  // check if mav would crash into ground
  if (wp3_z_ < 0) {
    ROS_WARN("Crashes onto ground - not executing!");
    return false;
  }

  if (wp3_z_ > 0.3) {
    ROS_WARN("Drop point too high, potential risk to damage to gps box - not executing!");
    return false;
  }

  mav_trajectory_generation::Vertex::Vector vertices;
  const int dimension = 3;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
  // we have 2 vertices: start = current position || end = Final point.
  mav_trajectory_generation::Vertex start(dimension), end(dimension);

  // set start point constraints
  // (current position, and everything else zero)
  start.makeStartOrEnd(current_position_.translation(), derivative_to_optimize);
  vertices.push_back(start);

  // plan final point if needed (to end position at rest).
  Eigen::Vector3d end_point_position = current_position_.translation();
  end_point_position.x() = current_position_.translation().x();
  end_point_position.y() = current_position_.translation().y();
  end_point_position.z() = wp3_z_;
  end.makeStartOrEnd(end_point_position, derivative_to_optimize);
  vertices.push_back(end);

  // compute segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, v_max_*0.2, a_max_);

  // solve trajectory
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.solveLinear();

  // get trajectory
  trajectory_.clear();
  opt.getTrajectory(&trajectory_);
  
  visualizeCachedTrajectory();
  return true;
}

bool TrajectoryPlanner::homecomingCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {

  mav_trajectory_generation::Vertex::Vector vertices;
  const int dimension = 3;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
  // we have 2 vertices: start = current position || end = Final point.
  mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

  // set start point constraints
  // (current position, and everything else zero)
  start.makeStartOrEnd(current_position_.translation(), derivative_to_optimize);
  vertices.push_back(start);

  // set middle point constraints
  Eigen::Vector3d middle_point_position = current_position_.translation();
  middle_point_position.x() = (wp2_x_ + sp_x_) / 2;
  middle_point_position.y() = (wp2_y_ + sp_y_) / 2;
  middle_point_position.z() = wp1_z_;
  middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, middle_point_position);
  vertices.push_back(middle);

  // plan final point if needed (to end position at rest).
  Eigen::Vector3d end_point_position = current_position_.translation();
  end_point_position.x() = sp_x_;
  end_point_position.y() = sp_y_;
  end_point_position.z() = (wp1_z_+sp_z_) / 2;
  end.makeStartOrEnd(end_point_position, derivative_to_optimize);
  vertices.push_back(end);

  // compute segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, v_max_, a_max_);

  // solve trajectory
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.solveLinear();

  // get trajectory
  trajectory_.clear();
  opt.getTrajectory(&trajectory_);
  
  visualizeCachedTrajectory();
  return true;
}

bool TrajectoryPlanner::visualizeCachedTrajectory() {
  visualization_msgs::MarkerArray markers;
  double distance =
      0.3; // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string frame_id = "world";

  mav_trajectory_generation::drawMavTrajectory(trajectory_,
                                               distance,
                                               frame_id,
                                               &markers);
  pub_markers_.publish(markers);
  return true;
}

bool TrajectoryPlanner::executeCachedTrajectory(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  mav_planning_msgs::PolynomialTrajectory4D msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory_, &msg);
  msg.header.frame_id = "world";
  pub_trajectory_.publish(msg);
  return true;
}
