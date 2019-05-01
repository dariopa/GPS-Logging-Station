#include <iostream>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

class TrajectoryPlanner {
 public:
  TrajectoryPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  void loadParameters();

  void uavPoseCallback(const geometry_msgs::Pose::ConstPtr& pose);

  void getFirstPose();

  bool takeoffCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  bool traverseCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  bool releaseCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  bool homecomingCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  // Visualize the last planned trajectory
  bool visualizeCachedTrajectory();

  // Output last planned trajectory
  bool executeCachedTrajectory(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  private:
  // Publisher
  ros::Publisher pub_markers_;
  ros::Publisher pub_trajectory_;

  // Subscriber
  ros::Subscriber sub_pose_;

  // Services
  ros::ServiceServer takeoff_service_;
  ros::ServiceServer traverse_service_;
  ros::ServiceServer release_service_;
  ros::ServiceServer homecoming_service_;
  ros::ServiceServer visualize_service_;
  ros::ServiceServer execute_service_;

  ros::NodeHandle& nh_;
  ros::NodeHandle& nh_private_;
  Eigen::Affine3d current_position_;
  mav_trajectory_generation::Trajectory trajectory_;

  // Startpoints
  double sp_x_;
  double sp_y_;
  double sp_z_;

  // Parameters
  double wp1_z_;
  double wp2_x_;
  double wp2_y_;
  double wp3_z_;
  double v_max_; // m/s
  double a_max_; // m/s^2
  double safety_altitude_; // m above take-off height.

};