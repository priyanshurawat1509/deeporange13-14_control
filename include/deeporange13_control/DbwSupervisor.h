/*
 A class deifinition to allow the supervising of dbw control and communication ...
between ROS and Raptor
This takes into account inputs from VehicleModel, dbwCan and rosHealth objects
to ensure safe and synced communication with Raptor.
*/


#ifndef _DBW_SUPERVISOR_H_
#define _DBW_SUPERVISOR_H_

#include <string.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <can_msgs/Frame.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <deeporange13_msgs/RaptorState.h>
#include <deeporange13_msgs/RosState.h>
#include <deeporange13_msgs/TrackVelocity.h>
#include <deeporange13_msgs/RosHealth.h>
#include <deeporange13_control/state_enums.h>

namespace deeporange_dbw_ros
{
    class DbwSupervisor
    {
        public:
        DbwSupervisor(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
        ~DbwSupervisor();

        private:
        void getCommandedTwist(const geometry_msgs::TwistStamped::ConstPtr& msg);
        void publishROSState(const ros::TimerEvent& event);
        void getRaptorMsg(const deeporange13_msgs::RaptorState& msg);
        void updateROSStateMsg();
        void ROSHealthCallback(const deeporange13_msgs::RosHealth& msg);

        ros::Timer timer_;

        // Publishers
        ros::Publisher pub_trackVel_;
        ros::Publisher pub_rosState_;
        ros::Publisher pub_estop_;
        // Subscribers
        ros::Subscriber sub_raptor_;
        ros::Subscriber sub_cmdVel_;
        ros::Subscriber sub_rosHealth_;


        // Init the msg variables
        nav_msgs::Odometry odometryMsg_;
        deeporange13_msgs::RaptorState raptorMsg_;
        deeporange13_msgs::RosState rosSupMsg_;
        deeporange13_msgs::TrackVelocity trackVelMsg_;
        geometry_msgs::TwistStamped commandedTwist_;

        // Setting up internal semaphores:
        bool is_raptorMsg_old_;
        int rosHealth_;
        bool useROSHealth_;

    };
}

#endif // _DBW_SUPERVISOR_H_