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
  
  // trajectory server
  trajectory_service_ = nh.advertiseService("trajectory", &TrajectoryPlanner::trajectoryCallback, this);
}

void TrajectoryPlanner::loadParameters() {
  CHECK(nh_private_.getParam("wp1_z", waypoint_1_z_) &&
        nh_private_.getParam("wp2_x", waypoint_2_x_) && 
        nh_private_.getParam("wp2_y", waypoint_2_y_) && 
        nh_private_.getParam("wp3_z", waypoint_3_z_) &&
        nh_private_.getParam("v_max", v_max_) &&
        nh_private_.getParam("a_max", a_max_) &&
        nh_private_.getParam("safety_altitude", safety_altitude_) &&
        nh_private_.getParam("approach_distance", approach_distance_) &&
        nh_private_.getParam("tolerance_distance", tolerance_distance_))
        << "Error loading parameters!";
}

void TrajectoryPlanner::uavPoseCallback(const geometry_msgs::Pose::ConstPtr& pose) {
  tf::poseMsgToEigen(*pose, current_position_);
}

void TrajectoryPlanner::getFirstPose() {
  startpoint_.translation() = current_position_.translation();
}

bool TrajectoryPlanner::checkPosition(double x_pos, double y_pos, double z_pos) {
  while(true) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    distance_to_goal_ = sqrt(pow(current_position_.translation().x() - x_pos, 2) + 
                             pow(current_position_.translation().y() - y_pos, 2) + 
                             pow(current_position_.translation().z() - z_pos, 2));
    if (distance_to_goal_ <= tolerance_distance_) {
      break;
    }
  }
  ROS_WARN("TRAJECTORY TERMINATED.");
  return true;
}

bool TrajectoryPlanner::trajectoryPlanner(double x_pos, double y_pos, double z_pos, double velocity, double accel) {
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
  end_point_position.x() = x_pos;
  end_point_position.y() = y_pos;
  end_point_position.z() = z_pos;
  end.makeStartOrEnd(end_point_position, derivative_to_optimize);
  vertices.push_back(end);

  // compute segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, velocity, accel);

  // solve trajectory
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.solveLinear();

  // get trajectory
  trajectory_.clear();
  opt.getTrajectory(&trajectory_);
  
  visualizeTrajectory();
  return true;
}

bool TrajectoryPlanner::trajectoryCallback(mav_drop_recovery::SetTargetPosition::Request& request, 
                                           mav_drop_recovery::SetTargetPosition::Response& response) {
  // TAKEOFF                                          
  if (request.command == "takeoff") {
    takeoff();
  }
  // TRAVERSE
  else if (request.command == "traverse") {
    traverse();
  }
  // RELEASE
  else if (request.command == "release") {
    release();
  }
  // RECOVERY
  else if (request.command == "recovery") {
    recovery();
  }
  // HOMECOMING
  else if (request.command == "homecoming") {
    homecoming();
  }
  else {
    ROS_WARN("INCORRECT_INPUT - CHECK AND RETRY");
    response.success == false;
    return false; 
  }

  // Check if trajectory execution is demanded.
  if (request.execute == true) {
    executeTrajectory();
    ROS_WARN("TRAJECTORY IS BEING EXECUTED!");
  }

  response.success == true;
  return true;
}

bool TrajectoryPlanner::takeoff() {
  double waypoint_x = current_position_.translation().x();
  double waypoint_y = current_position_.translation().y();

  // check if actually a take off
  if (waypoint_1_z_ < 0) {
    ROS_WARN("Not a takeoff - not executing!");
    return false;
  }
  // check if takeoff altitude is above safety altitude
  if (waypoint_1_z_ < safety_altitude_) {
    ROS_WARN("Take off too low. Increase takeoff altitude - not executing!");
    return false;
  }

  trajectoryPlanner(waypoint_x ,waypoint_y, waypoint_1_z_, v_max_*0.2, a_max_ * 0.3);
  return true;
}

bool TrajectoryPlanner::traverse() {
  double waypoint_z = current_position_.translation().z(); 
  
  trajectoryPlanner(waypoint_2_x_, waypoint_2_y_, waypoint_z, v_max_, a_max_);
  return true;
}

bool TrajectoryPlanner::release() {
  double waypoint_x = current_position_.translation().x();
  double waypoint_y = current_position_.translation().y();

  // check if mav would crash into ground
  if (waypoint_3_z_ < 0) {
    ROS_WARN("Crashes onto ground - not executing!");
    return false;
  }
  if (waypoint_3_z_ > 0.3) {
    ROS_WARN("Drop point too high, potential risk to damage to gps box - not executing!");
    return false;
  }

  trajectoryPlanner(waypoint_x, waypoint_y, waypoint_3_z_, v_max_*0.2, a_max_);
  return true;
}

bool TrajectoryPlanner::recovery() {
  double wp_x_approach = waypoint_2_x_ * (1 - approach_distance_/(sqrt(pow(waypoint_2_x_, 2) + pow(waypoint_2_y_, 2))));
  double wp_y_approach = waypoint_2_y_ * (1 - approach_distance_/(sqrt(pow(waypoint_2_x_, 2) + pow(waypoint_2_y_, 2))));

  trajectoryPlanner(wp_x_approach, wp_y_approach, waypoint_3_z_ + 0.3, v_max_ * 0.1, a_max_);

  trajectoryPlanner(waypoint_2_x_, waypoint_2_y_, waypoint_3_z_ + 0.2, v_max_ * 0.1, a_max_);

  trajectoryPlanner(waypoint_2_x_, waypoint_2_y_, waypoint_1_z_/2, v_max_ * 0.1, a_max_);
  return true;
}

bool TrajectoryPlanner::homecoming() {

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
  middle_point_position.x() = (waypoint_2_x_ + startpoint_.translation().x()) / 2;
  middle_point_position.y() = (waypoint_2_y_ + startpoint_.translation().y()) / 2;
  middle_point_position.z() = waypoint_1_z_;
  middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, middle_point_position);
  vertices.push_back(middle);

  // plan final point if needed (to end position at rest).
  Eigen::Vector3d end_point_position = current_position_.translation();
  end_point_position.x() = startpoint_.translation().x();
  end_point_position.y() = startpoint_.translation().y();
  end_point_position.z() = (waypoint_1_z_+startpoint_.translation().z()) / 2;
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
  
  visualizeTrajectory();
  return true;
}

bool TrajectoryPlanner::visualizeTrajectory() {
  visualization_msgs::MarkerArray markers;
  double distance = 0.3; // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string frame_id = "world";

  mav_trajectory_generation::drawMavTrajectory(trajectory_,
                                               distance,
                                               frame_id,
                                               &markers);
  pub_markers_.publish(markers);
  return true;
}

bool TrajectoryPlanner::executeTrajectory() {
  mav_planning_msgs::PolynomialTrajectory4D msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory_, &msg);
  msg.header.frame_id = "world";
  pub_trajectory_.publish(msg);
  return true;
}