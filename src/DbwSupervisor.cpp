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
        sub_raptor_ = node.subscribe("raptor_state", 10, &DbwSupervisor::getRaptorMsg, this, ros::TransportHints().tcpNoDelay(true));
        sub_cmdVel_ = node.subscribe("cmd_vel", 10, &DbwSupervisor::getCommandedTwist, this, ros::TransportHints().tcpNoDelay(true));
        sub_rosHealth_ = node.subscribe("ros_health", 10, &DbwSupervisor::ROSHealthCallback, this, ros::TransportHints().tcpNoDelay(true));

        pub_trackVel_ = node.advertise<can_msgs::Frame>("can_tx", 10);
        pub_estop_ = node.advertise<std_msgs::Bool>("fort_estop", 10);
        pub_rosState_ = node.advertise<deeporange13_msgs::RosState>("ros_state", 10);

        // Set up Timer - with calback to publish ROS state all the time that the node is running
        timer_ = node.createTimer(ros::Duration(1 / 20.0), &DbwSupervisor::publishROSState, this);

        /* Initiate ROS State with a NOT OK state to be safe. This state will be published till the ...
        ROS supervisor intentionally changes it.
        */
        rosSupMsg_.ros_state = AU_NOT_OK;

        priv_nh.getParam("use_ros_health", useROSHealth_);
    }

    DbwSupervisor::~DbwSupervisor(){}

    void DbwSupervisor::publishROSState(const ros::TimerEvent& event)
    {
        // Always continue to publish ROS state
        DbwSupervisor::updateROSStateMsg();
        pub_rosState_.publish(rosSupMsg_);
    }

    void DbwSupervisor::updateROSStateMsg()
    {
        switch (useROSHealth_)
        {
            case true:
            {
                if (rosHealth_ == HEALTH_OK)
                {
                    if (rosSupMsg_.ros_state == AU_NOT_OK)
                    {
                        /* Only shift to AU_OK state if zero velocity published by local controller. 
                        this ensures dbw-ros mode always starts safely
                        */
                        if (commandedTwist_.twist.linear.x == 0 && commandedTwist_.twist.angular.z == 0)
                        {
                            rosSupMsg_.ros_state == AU_OK;
                        }
                        else
                        {
                            rosSupMsg_.ros_state == AU_NOT_OK;
                        }
                    }
                    else if (rosSupMsg_.ros_state == AU_OK)
                        /* Keep it as isframe_ = message->GetFrame();
                    */
                    {
                        rosSupMsg_.ros_state == AU_OK;
                    }
                    else
                    // probably an erroneous/empty msg
                    {
                        rosSupMsg_.ros_state == AU_NOT_OK;
                    }
                    
                }
                else
                    // Health not ok
                {
                    rosSupMsg_.ros_state == AU_NOT_OK;
                }
            }
            case false:
            {
                if (rosSupMsg_.ros_state == AU_NOT_OK)
                {
                    /* Only shift to AU_OK state if zero velocity published by local controller. 
                    this ensures dbw-ros mode always starts safely
                    */
                    if (commandedTwist_.twist.linear.x == 0 && commandedTwist_.twist.angular.z == 0)
                    {
                        rosSupMsg_.ros_state == AU_OK;
                    }
                    else
                    {
                        rosSupMsg_.ros_state == AU_NOT_OK;
                    }
                }
                else if (rosSupMsg_.ros_state == AU_OK)
                    /* Keep it as is
                    */
                {
                    rosSupMsg_.ros_state == AU_OK;
                }
                /* No other case can exist since we init the rosSupMsg_.ros_state as AU_NOT_OK.
                But we add the following for safety.
                */
                else
                // probably an erroneous/empty msg
                {
                    rosSupMsg_.ros_state == AU_NOT_OK;
                }

            }
        }

        rosSupMsg_.header.stamp = ros::Time::now();
        /* 
        How else to decide if AU_OK? - 'ros_health' is the major parameter for this. 
        - See if the phoenix nodes that must run are running (this can be done by ros_health monitoring node)
        - Right now, raptor state does not decide ros state at all, which is OK since raptor is coded ...
            considering that. It doesnt send any acknowledge request to ROS.
        */
    
    }

    void DbwSupervisor::ROSHealthCallback(const deeporange13_msgs::RosHealth& msg)
    {
        /* get ros health topic from ros_monitor node and use it to decide if its safe to send track_vel to Raptor.
           This will be published by a separate node.
           Details of how and what all will be monitored is yet to be cleared.
        */
        rosHealth_ = msg.ros_health;

    }

    void DbwSupervisor::getCommandedTwist(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        commandedTwist_.twist.linear.x = msg->twist.linear.x;
        commandedTwist_.twist.angular.z = msg->twist.angular.z;
    }

    void DbwSupervisor::getRaptorMsg(const deeporange13_msgs::RaptorState& msg)
    {
        // check if the msg is old.
        if (msg.header.stamp < ros::Time::now())
        {
            ROS_WARN("Time stamp received from Raptor way too old. Not safe. Sending Zero cmd_vels now.");
            is_raptorMsg_old_ = true;
        }

        raptorMsg_.system_state = msg.system_state;
        // Do nothing with this information as of now.
    }

} // end namespace namespace deeporange_dbw_ros
