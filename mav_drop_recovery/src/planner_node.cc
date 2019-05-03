/*
 * Simple code for structure approval, only takeoff implemented, 
 * no logic so far. 
 *
 *-> open terminal
 * 
 * Launch via
 *   roslaunch mav_drop_recovery takeoff.launch
 *
 * -> open new terminal
 * 
 * To call the takeoff service
 *   rosservice call /firefly/takeoff
 * 
 * To visualize trajectory
 *   rosservice call /firefly/visualize
 * 
 * To execute path
 *   rosservice call /firefly/execute
 */

#include  "ros/ros.h"
#include <mav_drop_recovery/planner.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "simple_planner");

  ros::NodeHandle n;
  ros::NodeHandle nh_private("~");
  TrajectoryPlanner planner(n, nh_private);
  ROS_WARN_STREAM("SLEEPING FOR 5s TO WAIT FOR CLEAR CONSOLE");
  ros::Duration(5.0).sleep();
  planner.loadParameters();
  planner.getFirstPose();
  ROS_WARN_STREAM("READY TO GO!");

  ros::spin();

  return 0;
}