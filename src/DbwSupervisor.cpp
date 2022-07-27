/*
 A class deifinition to allow the supervising of dbw control and communication ...
between ROS and Raptor
This takes into account inputs from VehicleModel, dbwCan and rosHealth objects
to ensure safe and synced communication with Raptor.
*/

#include <deeporange13_control/DbwSupervisor.h>

namespace deeporange_dbw_ros
{
    DbwSupervisor::DbwSupervisor(ros::NodeHandle &node, ros::NodeHandle &priv_nh)
    {
        // Instantiate sub/pubs
        sub_raptor_ = node.subscribe("/deeporange_dbw_ros/raptor_state", 10, &DbwSupervisor::getRaptorMsg, this, ros::TransportHints().tcpNoDelay(true));
        sub_cmdVel_ = node.subscribe("/deeporange1314/cmd_vel", 10, &DbwSupervisor::getCommandedTwist, this, ros::TransportHints().tcpNoDelay(true));
        sub_rosHealth_ = node.subscribe("/deeporange_dbw_ros/ros_health", 10, &DbwSupervisor::ROSHealthCallback, this, ros::TransportHints().tcpNoDelay(true));

        pub_trackVel_ = node.advertise<can_msgs::Frame>("can_tx", 10);
        pub_estop_ = node.advertise<std_msgs::Bool>("fort_estop", 10);
        pub_rosState_ = node.advertise<deeporange13_msgs::RosState>("/deeporange_dbw_ros/ros_state", 10);

        // Set up Timer - with calback to publish ROS state all the time that the node is running
        timer_ = node.createTimer(ros::Duration(1.0 / 20.0), &DbwSupervisor::publishROSState, this);

        /* Initiate ROS State with a DEFAULT state to be safe. This state will be published till the ...
        ROS supervisor intentionally changes it.
        */
        rosSupMsg_.ros_state = AU0_DEFAULT;
        rosSupMsg_.raptor_state = SS1_DEFAULT;
        rosSupMsg_.counter = 1;
        rosSupMsg_.ros_health = HEALTH0_DEFAULT;
        is_Phx_NavigationActive_ = false;
        
    }

    DbwSupervisor::~DbwSupervisor(){}

    void DbwSupervisor::publishROSState(const ros::TimerEvent& event)
    {
        /* Always continue to publish ROS state.
        This is the heartbeat signal
        */
        DbwSupervisor::updateROSStateMsg();
        pub_rosState_.publish(rosSupMsg_);
    }

    void DbwSupervisor::updateROSStateMsg()
    {
        if (rosSupMsg_.ros_health == HEALTH3_OK || rosSupMsg_.ros_health == HEALTH2_WARN)
        {
            if (rosSupMsg_.ros_state == AU0_DEFAULT)
            {
                /* this means it is in starting state.
                */
                if (commandedTwist_.linear.x == 0 && commandedTwist_.angular.z == 0 && is_Phx_NavigationActive_ == true)
                {
                    rosSupMsg_.ros_state = AU2_NAV_ACTIVE;
                }
                else if (is_Phx_NavigationActive_ == false)
                {
                    rosSupMsg_.ros_state = AU3_NAV_INACTIVE;
                    ROS_INFO("Stack navigation is not publishing cmd_vels");
                }
                else
                {
                    rosSupMsg_.ros_state = AU3_NAV_INACTIVE;
                    ROS_INFO("There is a non-zero cmd_vel already being published. You do not want to start navigation like this");
                }

            }
            if (rosSupMsg_.ros_state == AU3_NAV_INACTIVE)
            {
                /* Only shift to AU2_NAV_ACTIVE state if zero velocity published by local controller. 
                this ensures dbw-ros mode always starts safely
                */
                if (commandedTwist_.linear.x == 0 && commandedTwist_.angular.z == 0 && is_Phx_NavigationActive_ == true)
                {
                    rosSupMsg_.ros_state = AU2_NAV_ACTIVE;
                }
                else if (is_Phx_NavigationActive_ == false)
                {
                    rosSupMsg_.ros_state = AU3_NAV_INACTIVE;
                    ROS_INFO("Stack navigation is not publishing cmd_vels");
                }
                else
                {
                    rosSupMsg_.ros_state = AU3_NAV_INACTIVE;
                    ROS_INFO("There is a non-zero cmd_vel already being published. You do not want to start navigation like this");
                }
            }
            else if (rosSupMsg_.ros_state == AU2_NAV_ACTIVE)
                /* Keep it as is till phx navigation is active
            */
            {
                if (is_Phx_NavigationActive_ == true)
                {
                    rosSupMsg_.ros_state = AU2_NAV_ACTIVE;
                }
                else
                {
                    rosSupMsg_.ros_state = AU3_NAV_INACTIVE;
                }
            }
            else
            // probably an erroneous/empty msg
            {
                rosSupMsg_.ros_state = AU3_NAV_INACTIVE;
                ROS_WARN("The message object 'ros_state' is empty/erroneous right now. Code should not have arrived here");
            }
            
        }
        else
            // Health - HEALTH_FATAL or unkwown value
        {
            rosSupMsg_.ros_state = AU1_NOT_OK;
            // ROS_INFO("%d ROS STATE", rosSupMsg_.ros_state);
            ROS_INFO("Health is either HEALTH1_FATAL or HEALTH0_DEFAULT ");
        }

        /* Run a counter for handshake/heartbeat 
         from 1 to 5
        */
        if (rosSupMsg_.counter < 5 )
        {
            rosSupMsg_.counter += 1;
        }
        else
        {
            rosSupMsg_.counter = 1;
        }

        // Add timestamp
        rosSupMsg_.header.stamp = ros::Time::now();
        /*
        How else to decide if AU2_NAV_ACTIVE? - 'ros_health' is the major parameter for this. 
        - See if the phoenix nodes that must run are running (this can be done by ros_health monitoring node)
        - Right now, raptor state does not decide ros state at all, which is OK since raptor is coded ...
            considering that. It doesnt send any acknowledge request to ROS.
        */
    
    }

    void DbwSupervisor::ROSHealthCallback(const deeporange13_msgs::RosHealth::ConstPtr& msg)
    {
        /* get ros health topic from ros_monitor node and use it to decide if its safe to send track_vel to Raptor.
           This will be published by a separate node.
           Details of how and what all will be monitored is yet to be cleared.
        */
        rosSupMsg_.ros_health = msg->ros_health;
        if (msg->phoenix_status.type == 2)
        {
            is_Phx_NavigationActive_ = true;
        }
        else        
        {
            is_Phx_NavigationActive_ = false;
        }        

    }

    void DbwSupervisor::getCommandedTwist(const geometry_msgs::Twist::ConstPtr& msg)
    {
        commandedTwist_.linear.x = msg->linear.x;
        commandedTwist_.angular.z = msg->angular.z;
    }

    void DbwSupervisor::getRaptorMsg(const deeporange13_msgs::RaptorState::ConstPtr& msg)
    {
        // check if the msg is old.
        if (msg->header.stamp < ros::Time::now())
        {
            // ROS_WARN("Time stamp received from Raptor way too old. Not safe. Sending Zero cmd_vels now.");
            is_raptorMsg_old_ = true;
        }

        raptorMsg_.system_state = msg->system_state;
        rosSupMsg_.raptor_state = msg->system_state;
        // Do nothing with this information as of now.
    }

//    void DbwSupervisor::checkRaptorHeartbeart(const int counter)
//    {
//        ros::Time time0 = ros::Time::now();
//
//    }

} // end namespace namespace deeporange_dbw_ros


/*
Notes on states:
1. AU2_NAV_ACTIVE is the only one that raptor will need to stay in dbwMode_ROS. 
2. So the idea is to set this state only and only when phx is publishing cmd-vels
3. set to AU3_NAV_INACTIVE - when no cmd-vel generated - for whatever reason (whether phx is running without "navigation" functions)
4. AU1_NOT_OK - is set when ros_health is HEALTH1_FATAL.
*/