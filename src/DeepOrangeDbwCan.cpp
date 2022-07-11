/* 
A class to manage can msgs (parse and publish to can_tx/rx) using dbc file provided
Relies on DbwSupervisor to figure out what to send to Raptor. Uses "ros_state" topic to get this info
Receives CAN data from socketcan node and provides info to DbwSupervisor
*/

#include <deeporange13_control/DeepOrangeDbwCan.h>

namespace deeporange_dbw_ros
{
    DeepOrangeDbwCan::DeepOrangeDbwCan(ros::NodeHandle &node, ros::NodeHandle &priv_nh)
    {
        // Initialise variables
        estopMsg_.data = 0;

        // Instantiate sub/pubs
        sub_can_ = node.subscribe("can_tx", 10, &DeepOrangeDbwCan::recvCAN, this, ros::TransportHints().tcpNoDelay(true));
        sub_trackVel_ = node.subscribe("track_commands", 10, &DeepOrangeDbwCan::publishTrackCommandstoCAN, this, ros::TransportHints().tcpNoDelay(true));
        sub_rosState_ = node.subscribe("ros_state", 10, &DeepOrangeDbwCan::publishRosState, this, ros::TransportHints().tcpNoDelay(true));

        pub_raptorState_ = node.advertise<deeporange13_msgs::RaptorState>("raptor_state", 10);
        pub_can_ = node.advertise<can_msgs::Frame>("can_rx", 10);
        pub_estop_ = node.advertise<std_msgs::Bool>("fort_estop", 10);
        // pub_can_.publish(frame_);

        // Get params
        priv_nh.getParam("dbw_dbc_file", dbcFile_);

        // Instantiate the dbc class
        dbwDbc_ = NewEagle::DbcBuilder().NewDbc(dbcFile_);
    }

    DeepOrangeDbwCan::~DeepOrangeDbwCan(){} 

    void DeepOrangeDbwCan::recvCAN(const can_msgs::Frame::ConstPtr& msg)
    {
        if (!msg->is_rtr && !msg->is_error)
        {
            switch (msg->id)
            {
                case ID_VSS_MSG:
                 // getting raptor state and publishing to ROS
                {
                    NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_VSS_MSG);
                    if (msg->dlc >= message->GetDlc())
                    {
                        message->SetFrame(msg);
                        raptorMsg_.header.stamp = msg->header.stamp;
                        raptorMsg_.system_state = message->GetSignal("Sys_State")->GetResult();
                        pub_raptorState_.publish(raptorMsg_);
                    }
                }
                break;

                case ID_MOTOR1_POSITION:
                {
                    NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_MOTOR1_POSITION);
                    if (msg->dlc >= message->GetDlc())
                    {
                        odometryMsg_.header.stamp = msg->header.stamp;
                        /* TO DO:
                        Add code for generating ros topic of type nav_msgs/Odometry
                        using motor encoder data
                        */
                    }
                }
                break;

            }
        }
    }

    void DeepOrangeDbwCan::publishTrackCommandstoCAN(const deeporange13_msgs::TrackVelocity& msg)
    {
        if (rosSupMsg_.ros_state == AU_OK)
        /* Only publish mobility commands to raptor if ROS is OK
        Does not matter what raptor state is. No need to check it.
        Raptor Vehicle State Supervisor ensures that ROS messages are 
        accepted only in the DBW_ROS mode in SS8_NOMINALOP.
        But if architecture is changed in future, changes must be made here
        */
        {
            NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_ROS_CONTROL_MSG);
            message->GetSignal("Left_TrackTrq_Demand")->SetResult(0);
            message->GetSignal("Right_TrackTrq_Demand")->SetResult(0);
            message->GetSignal("Left_TrackVel_Demand")->SetResult(msg.angular.leftTrackSpeed);
            message->GetSignal("Right_TrackVel_Demand")->SetResult(msg.angular.rightTrackSpeed);
            frame_ = message->GetFrame();
            pub_can_.publish(frame_);
        }
        // else, do not publish track commands.
    }
    
    void DeepOrangeDbwCan::publishRosState(const deeporange13_msgs::RosState& msg)
    {
        // check for time stamp? Should be relevant?
        // rosSupMsg_.header.stamp = msg.header.stamp;

        // Get ros state from DbwSupervisor and send to can whenever received.
        NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_ROS_SUP_MSG);
        message->GetSignal("Au_State")->SetResult(msg.ros_state);
        message->GetSignal("ROS_Health")->SetResult(0); // update if needed to be sent to raptor. otherwise remove.
        frame_ = message->GetFrame();
        pub_can_.publish(frame_);
    }


} // end namespace deeporange_dbw_ros
