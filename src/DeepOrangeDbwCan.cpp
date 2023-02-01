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
        // sub_trackVel_ = node.subscribe("/deeporange_dbw_ros/track_commands", 10, &DeepOrangeDbwCan::publishTrackCommandstoCAN, this, ros::TransportHints().tcpNoDelay(true));
        sub_rosState_ = node.subscribe("/deeporange_dbw_ros/ros_state", 10, &DeepOrangeDbwCan::publishRosState, this, ros::TransportHints().tcpNoDelay(true));
        sub_cmdVel_ = node.subscribe("/deeporange1314/cmd_vel", 10, &DeepOrangeDbwCan::publishVelocitytoCAN, this, ros::TransportHints().tcpNoDelay(true));
        sub_cmdTq = node.subscribe("/deeporange1314/cmd_tq", 10, &DeepOrangeDbwCan::publishTorquetoCAN, this, ros::TransportHints().tcpNoDelay(true));
        pub_raptorState_ = node.advertise<deeporange13_msgs::RaptorState>("/deeporange_dbw_ros/raptor_state", 10);
        pub_can_ = node.advertise<can_msgs::Frame>("can_rx", 10);
        pub_estop_ = node.advertise<std_msgs::Bool>("fort_estop", 10);

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

                // case :
                // for others - none currently

            }
        }
    }

    void DeepOrangeDbwCan::publishTrackCommandstoCAN(const deeporange13_msgs::TrackVelocity& msg)
    // Get Track commands and publish to CAN
    {
        NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_ROS_CONTROL_VELOCITY_MSG);
        if (rosSupMsg_.ros_state == AU2_NAV_ACTIVE)
        /* publish mobility commands to raptor 
        Does not matter what raptor state is. No need to check it.
        Raptor Vehicle State Supervisor ensures that ROS messages are 
        accepted only in the DBW_ROS mode in SS8_NOMINALOP.
        But if architecture is changed in future, changes must be made here
        */
        {
            message->GetSignal("Left_TrackVel_Demand")->SetResult(msg.linear.leftTrackSpeed*1000);
            message->GetSignal("Right_TrackVel_Demand")->SetResult(msg.linear.rightTrackSpeed*1000);
        }
        else
        {
            ROS_WARN("Code should not have reached here. AU or VSS not in acceptable state for ROS track commands to go through.");
            /* Send zero track commands to CAN to be safe. 
            Although Raptor will not be in DbwMode_ROS at this point
            */
            message->GetSignal("Left_TrackVel_Demand")->SetResult(0);
            message->GetSignal("Right_TrackVel_Demand")->SetResult(0);
        }
        frame_ = message->GetFrame();
        pub_can_.publish(frame_);
    }

    void DeepOrangeDbwCan::publishTorquetoCAN(const deeporange13_msgs::TorqueValues& msg)
    // get cmd_vel and publish to CAN
    {
        NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_ROS_CONTROL_TORQUE_MSG);
        if (rosSupMsg_.ros_state == AU2_NAV_ACTIVE)
        /* publish mobility commands to raptor 
        Does not matter what raptor state is. No need to check it.
        Raptor Vehicle State Supervisor ensures that ROS messages are 
        accepted only in the DBW_ROS mode in SS8_NOMINALOP.
        But if architecture is changed in future, changes must be made here
        */
        {
            message->GetSignal("Left_TrackTrq_Demand")->SetResult(msg.leftTorque);
            message->GetSignal("Right_TrackTrq_Demand")->SetResult(msg.rightTorque);
        }
        else
        {
            ROS_WARN("Code should not have reached here. AU or VSS not in acceptable state for ROS track commands to go through.");
            /* Send zero track commands to CAN to be safe. 
            Although Raptor will not be in DbwMode_ROS at this point
            */
            message->GetSignal("Left_TrackTrq_Demand")->SetResult(0);
            message->GetSignal("Right_TrackTrq_Demand")->SetResult(0);
        }
        frame_ = message->GetFrame();
        pub_can_.publish(frame_);
    }    
    
    void DeepOrangeDbwCan::publishVelocitytoCAN(const geometry_msgs::Twist::ConstPtr& msg)
    // get cmd_vel and publish to CAN
    {
        NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_ROS_CONTROL_MSG);
        //NewEagle::DbcMessage* message1 = dbwDbc_.GetMessageById(ID_ROS_CONTROL_TEST);
        if (rosSupMsg_.ros_state == AU2_NAV_ACTIVE)
        /* publish mobility commands to raptor 
        Does not matter what raptor state is. No need to check it.
        Raptor Vehicle State Supervisor ensures that ROS messages are 
        accepted only in the DBW_ROS mode in SS8_NOMINALOP.
        But if architecture is changed in future, changes must be made here
        */
        {
            message->GetSignal("Linear_X_Demand")->SetResult(msg->linear.x*(1000));
            message->GetSignal("Angular_Z_Demand")->SetResult(msg->angular.z*(1000));
            //message1->GetSignal("Linear_X_Demand")->SetResult(msg->linear.x*(1000));
            //message1->GetSignal("Angular_Z_Demand")->SetResult(msg->angular.z*(1000));
        }
        else
        {
            ROS_WARN("Code should not have reached here. AU or VSS not in acceptable state for ROS track commands to go through.");
            /* Send zero track commands to CAN to be safe. 
            Although Raptor will not be in DbwMode_ROS at this point
            */
            message->GetSignal("Linear_X_Demand")->SetResult(0);
            message->GetSignal("Angular_Z_Demand")->SetResult(0);
            //message1->GetSignal("Linear_X_Demand")->SetResult(0);
            //message1->GetSignal("Angular_Z_Demand")->SetResult(0);
        }
        frame_ = message->GetFrame();
        pub_can_.publish(frame_);
        //frame_ = message1->GetFrame();
        //pub_can_.publish(frame_);
    }    

    void DeepOrangeDbwCan::publishRosState(const deeporange13_msgs::RosState& msg)
    {
        // check for time stamp? Should be relevant?
        rosSupMsg_.header.stamp = msg.header.stamp;
        rosSupMsg_.ros_state = msg.ros_state;
        rosSupMsg_.raptor_state = msg.raptor_state;
        // Get ros state from DbwSupervisor and send to can whenever received.
        NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_ROS_SUP_MSG);
        message->GetSignal("Au_State")->SetResult(msg.ros_state);
        message->GetSignal("ROS_Health")->SetResult(rosSupMsg_.ros_health); // update if needed to be sent to raptor. otherwise remove.
        frame_ = message->GetFrame();
        pub_can_.publish(frame_);
    }


} // end namespace deeporange_dbw_ros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
