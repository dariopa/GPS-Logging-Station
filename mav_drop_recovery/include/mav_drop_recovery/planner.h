#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

#include "mav_drop_recovery/SetTargetPosition.h"

class TrajectoryPlanner {
 public:
  TrajectoryPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  void loadParameters();

  void uavPoseCallback(const geometry_msgs::Pose::ConstPtr& pose);

  void getFirstPose();

  bool checkPosition(double x_pos, double y_pos, double z_pos);

  bool trajectoryPlanner(double x_pos, double y_pos, double z_pos, double velocity, double accel);

  // Service caller for trajectory
  bool trajectoryCallback(mav_drop_recovery::SetTargetPosition::Request& request, 
                          mav_drop_recovery::SetTargetPosition::Response& response);

  // Different trajectories
  bool takeoff();

  bool traverse();

  bool release();

  bool recovery();

  bool homecoming();

  // Visualize the last planned trajectory
  bool visualizeTrajectory();

  // Output last planned trajectory
  bool executeTrajectory();

  private:
  // Publisher
  ros::Publisher pub_markers_;
  ros::Publisher pub_trajectory_;

  // Subscriber
  ros::Subscriber sub_pose_;

  // Services
  ros::ServiceServer trajectory_service_;

  ros::NodeHandle& nh_;
  ros::NodeHandle& nh_private_;
  Eigen::Affine3d current_position_;
  mav_trajectory_generation::Trajectory trajectory_;

  // Startpoints
  Eigen::Affine3d startpoint_;
  double distance_to_goal_;

  // Parameters
  double waypoint_1_z_; // takeoff height
  double waypoint_2_x_; // x coord. for traverse
  double waypoint_2_y_; // y coord. for traverse
  double waypoint_3_z_; // release height
  double v_max_; // m/s
  double a_max_; // m/s^2
  double safety_altitude_; // m above take-off height.
  double approach_distance_; // distance from which gps will be approached
  double tolerance_distance_; // used in checkPosition();

};