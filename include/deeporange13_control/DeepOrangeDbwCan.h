/* 
A class to manage can msgs (parse and publish to can_tx/rx) using dbc file provided
Relies on DbwSupervisor to figure out what to send to Raptor. Uses "ros_state" topic to get this info
Receives CAN data from socketcan node and provides info to DbwSupervisor
*/

#ifndef _DEEPORANGE_DBW_CAN_H_
#define _DEEPORANGE_DBW_CAN_H_

#include <string.h>
#include <ros/ros.h>
// ROS messages
#include <can_msgs/Frame.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <deeporange13_msgs/RaptorState.h>
#include <deeporange13_msgs/RosState.h>
#include <deeporange13_msgs/TrackVelocity.h>
#include <deeporange13_msgs/Vector2.h>
#include <deeporange13_msgs/TorqueValues.h>
#include <deeporange13_msgs/MeasuredVelocity.h>
// Can dbc parser mackage
#include <can_dbc_parser/DbcMessage.h>
#include <can_dbc_parser/DbcSignal.h>
#include <can_dbc_parser/Dbc.h>
#include <can_dbc_parser/DbcBuilder.h>
// enumerations
#include <deeporange13_control/dispatch_can_msgs.h>
#include <deeporange13_control/state_enums.h>

namespace deeporange_dbw_ros

{
    class DeepOrangeDbwCan
    {
        public:
        DeepOrangeDbwCan(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
        ~DeepOrangeDbwCan();

        private:
        void recvCAN(const can_msgs::Frame::ConstPtr& msg);
        void publishCAN(const ros::TimerEvent& event);
        void publishTrackCommandstoCAN(const deeporange13_msgs::TrackVelocity& msg);
        void publishVelocitytoCAN(const geometry_msgs::Twist::ConstPtr& msg);
        void publishRosState(const deeporange13_msgs::RosState& msg);
        void publishTorquetoCAN(const deeporange13_msgs::TorqueValues& msg);
        void publishMeasuredVeltoCAN(const deeporange13_msgs::MeasuredVelocity& msg);
        void getMeasuredVx(const nav_msgs::Odometry& msg);
        void getMeasuredWz(const sensor_msgs::Imu& msg);

        // ros::Timer timer_;

        // Publishers
        ros::Publisher pub_can_;
        ros::Publisher pub_estop_;
        ros::Publisher pub_raptorState_;
        ros::Publisher pub_measuredVel_;

        // Subscribers
        ros::Subscriber sub_can_;
        ros::Subscriber sub_trackVel_;
        ros::Subscriber sub_rosState_;
        ros::Subscriber sub_cmdVel_;
        ros::Subscriber sub_cmdTq;
        ros::Subscriber sub_gpsImu_;
        ros::Subscriber sub_odom_;

        // Published msgs
        deeporange13_msgs::RaptorState raptorMsg_;
        deeporange13_msgs::MeasuredVelocity measuredVelocity_;
        
        // Subscribed msgs
        deeporange13_msgs::RosState rosSupMsg_;
        nav_msgs::Odometry odometryMsg_;
        sensor_msgs::Imu gpsImuMsg_;

        // Datatype for Wz
        std::vector<float> vectorWz_;
        float averageWz_ = 0;
        // float imutime_ = 0;

        // Datatype for Vx
        std::vector<float> vectorVx_;
        float averageVx_ = 0;
        // float odomtime_ = 0;
        
        // Frame ID
        std::string frameId_;
        can_msgs::Frame frame_;

        // E-stop indicator variables - unused
        long int estop_;
        std_msgs::Bool estopMsg_;

        // dbc file variables
        NewEagle::Dbc dbwDbc_;
        std::string dbcFile_;

        // Raptor Comms Variables - unused
        int sysState_;

    };
} // deeporange_dbw_ros

#endif // _DEEPORANGE_DBW_CAN_H_
