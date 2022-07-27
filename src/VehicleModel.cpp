/* A class that has definition of the vehicle model (tracked) and 
functions to split velocity to left and right track velocities or torques

Starting with a simple model. Add fidelity later
*/

#include <deeporange13_control/VehicleModel.h>

namespace  deeporange_dbw_ros
{
    VehicleModel::VehicleModel(ros::NodeHandle &node, ros::NodeHandle &priv_nh)
    {
        sub_cmdVel_ = node.subscribe("/deeporange1314/cmd_vel", 10, &VehicleModel::cmdVel2trackVel, this);
        pub_trackVel_ = node.advertise<deeporange13_msgs::TrackVelocity>("/deeporange_dbw_ros/track_commands", 10);
        // priv_nh.getParam("vehicle_trackwidth", trackWidth_);
        trackWidth_ = 2.4; // DO13 wheeltrack
    }

    VehicleModel::~VehicleModel(){}

    void VehicleModel::cmdVel2trackVel(const geometry_msgs::Twist::ConstPtr& msg)
    {
        // Get the velocity components form cmd_vel
        vX_ = msg->linear.x;
        wZ_ = msg->angular.z;

        //calculating the track velocity based on a simple no-slip kinematic model
        float vR = (2*vX_ + wZ_*trackWidth_)/2;
        float vL = (2*vX_ - wZ_*trackWidth_)/2;
        trackVelMsg_.linear.leftTrackSpeed = vL;
        trackVelMsg_.linear.rightTrackSpeed = vR;

        // publish zero rad/sec for now. To be updated
        trackVelMsg_.angular.leftTrackSpeed = 0;
        trackVelMsg_.angular.rightTrackSpeed = 0;

        // Publish
        pub_trackVel_.publish(trackVelMsg_);
    }
}