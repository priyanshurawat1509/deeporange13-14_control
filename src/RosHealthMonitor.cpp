/* 
A Class to monitor ROS health and provide info to DbwSuperisor for deciding AU_State on DO13
*/

#include <deeporange13_control/RosHealthMonitor.h>


namespace deeporange_dbw_ros
{

RosHealthMonitor::RosHealthMonitor(ros::NodeHandle& node, ros::NodeHandle &priv_nh)
{
    // tf_sub = n.subscribe("/rosout",100,&rosHealth::tfCallback,this);
    lidar_sub = node.subscribe("/deeporange1314/lidar_points", 100, &RosHealthMonitor::lidarCallback, this);
    rtk_sub = node.subscribe("/deeporange1314/novatel/oem7/inspvax", 1, &RosHealthMonitor::inspvaxCallback, this);
    sub_cmdVel_ = node.subscribe("/deeporange1314/cmd_vel", 10, &RosHealthMonitor::cmdVelCallback, this);

    health_pub = node.advertise<deeporange13_msgs::RosHealth>("/deeporange_dbw_ros/ros_health",100);

    //callback to publish the ROS-health state as soon as the node starts running
    timer = node.createTimer(ros::Duration(1.0 / 10.0), &RosHealthMonitor::publishROSHealth, this);
    
    //Initialise with default state for the ros_health
    healthMsg.ros_health = HEALTH0_DEFAULT;
}

RosHealthMonitor::~RosHealthMonitor(){};

//member function to publish the ROS health
void RosHealthMonitor::publishROSHealth(const ros::TimerEvent& event)
{
    /*the individual status checks are updated in their respective callbacks but not published, this function publishes them
    Additionally, it updates the 'ros_health' part of the message with the overall ROS health
    */ 
    if (healthMsg.rtk_status.type == HEALTH_ASPECT_NOT_FOUND || healthMsg.rtk_status.type == HEALTH_ASPECT_SEARCHING)
    {
        healthMsg.ros_health = HEALTH2_WARN;
        ROS_WARN("Please check for the GNSS solution available. It is not RTK-fixed");
    }
    else if (healthMsg.rtk_status.type == HEALTH_ASPECT_FOUND)
    {
        healthMsg.ros_health = HEALTH3_OK;
    }
    else
    {
        ROS_INFO("ROS health is in default state.");
    }

    // Check if phoenix stack is navigation active
    if (abs(phxLastCmdVelReceiveTime_.toSec() - ros::Time::now().toSec()) <= 1.0 )
    {
        // Stack is up publishing cmd_vel
        healthMsg.phoenix_status.type = HEALTH_ASPECT_FOUND;
    }
    else
    {
        healthMsg.phoenix_status.type = HEALTH_ASPECT_NOT_FOUND;
    }

    healthMsg.header.stamp = ros::Time::now();
    health_pub.publish(healthMsg);
}

//callback for PTP error -- checks for header messages in /tf and /lidar_points
void RosHealthMonitor::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    lidar_tSec = msg->header.stamp.sec;
}


//INSPVAX callback to check the available fix
void RosHealthMonitor::inspvaxCallback(const novatel_oem7_msgs::INSPVAX::ConstPtr& msg)
{
    if (msg->pos_type.type == 50 || msg->pos_type.type == 56) 
     //50 & 56 correspond to NARROW_INT & INS_RTKFIXED
    {
        if (rtk_checkStart_1 == 0)
                {
                    startTimer = ros::Time::now().toSec();
                    rtk_checkStart_1 = 1;
                    rtk_checkStart_2 = 0;
                }
                else
                {
                    if (ros::Time::now().toSec() - startTimer <= 5.0)
                    {
                        healthMsg.rtk_status.type = HEALTH_ASPECT_SEARCHING;
                    }
                    else
                    {
                        healthMsg.rtk_status.type = HEALTH_ASPECT_FOUND;
                    }
                }
    }
    else
    {
        if (rtk_checkStart_2 == 0)
        {
            
            startTimer = ros::Time::now().toSec();
            rtk_checkStart_1 = 0;
            rtk_checkStart_2 = 1;
        }
        else
        {
            if (ros::Time::now().toSec() - startTimer <= 5.0)
            {
                healthMsg.rtk_status.type = HEALTH_ASPECT_SEARCHING;
            }
            else
            {
                healthMsg.rtk_status.type = HEALTH_ASPECT_NOT_FOUND;
            }
        }
    }
    healthMsg.header.stamp = ros::Time::now();
}


void RosHealthMonitor::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // Stack is up and navigaton is active
    phxLastCmdVelReceiveTime_ = ros::Time::now();
}

}
// int main(int argc, char** argv){
//     ros::init(argc, argv,"ros_health");
//     ros::NodeHandle n;
//     RosHealthMonitor healthObj = RosHealthMonitor(n);

//     while (ros::ok()){

//         ros::spinOnce();
        
//     }
//     return 0;
// }

