/*********************************************************************
Written for use in Deep Orange 13 Drive-by-wire package
Makes use of New Eagle package: can_dbc_parser (https://github.com/NewEagleRaptor/raptor-dbw-ros/tree/master/can_dbc_parser)

 *********************************************************************/

#include <ros/ros.h>
#include <deeporange13_control/DeepOrangeDbwCan.h>
#include <deeporange13_control/VehicleModel.h>
#include <deeporange13_control/DbwSupervisor.h>
#include <deeporange13_control/RosHealthMonitor.h>
#include <deeporange13_control/StackVelPub.h>
#include <deeporange13_control/DataLogger.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "deeporange_dbw");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create CAN dbw object
  deeporange_dbw_ros::DeepOrangeDbwCan n_can(node, priv_nh);

  // create vehicle model object
  // deeporange_dbw_ros::VehicleModel n_dynamics(node, priv_nh);

  // create the dbwSupervisor object
  deeporange_dbw_ros::DbwSupervisor n_sup(node, priv_nh);

  // create ROS Health object
  deeporange_dbw_ros::RosHealthMonitor n_roshealth(node, priv_nh);

  // create Data Logger object
  deeporange_dbw_ros::DataLogger n_datalogger(node, priv_nh);

  /* only for stackless testing - 
  This object relies on tos topic /cmd_velocity_choice to decide what cmd_vel to send to CAN.
  */
  // VehicleMotionPublisher n_velPub(node, priv_nh);

  
  // handle callbacks until shut down
  ros::spin();

  return 0;
}
