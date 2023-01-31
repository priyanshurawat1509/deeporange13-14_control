/* 
    A class definition to automatically initiate Rosbag record (for certain
    topics) and CAN data logging (for all CAN buses together). Data logging 
    starts (if not already happening) if the Raptor ECU is in a system state
    from [SS2_WAKE, SS8_NOMINALOP]. The data logging (rosbag and CAN log) is
    stopped if the system state becomes SS_31 or an error state (>= 200) is
    reached.
*/

#include <deeporange13_control/DataLogger.h>

namespace deeporange_dbw_ros
{
    DataLogger::DataLogger(ros::NodeHandle &node, ros::NodeHandle &priv_nh)
    {
        // Obtain ros::Subscriber object
        sub_raptor_ = node.subscribe("/deeporange_dbw_ros/raptor_state", 10, &DataLogger::recordRosbagAndCANlog, this, ros::TransportHints().tcpNoDelay(true));
        
        // Initialize recording state to false
        isRecording = false;
    }
    
    DataLogger::~DataLogger(){}

    void DataLogger::recordRosbagAndCANlog(const deeporange13_msgs::RaptorState::ConstPtr& msg)
    {
        if (!isRecording && (msg->system_state == SS8_NOMINALOP))
        {
            // Obtaining timestamp (EST) to name ROS bag and CAN dump
            boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
            typedef boost::date_time::local_adjustor<boost::posix_time::ptime, -5, boost::posix_time::us_dst> us_eastern;
            my_posix_time = us_eastern::utc_to_local(my_posix_time);    // Conversion from UTC to EST (Clemson, South Carolina)
            std::string iso_time_str = boost::posix_time::to_iso_string(my_posix_time);
            std::string file_name_header = std::string("do13_") + iso_time_str.substr(0,4) + std::string("-") + iso_time_str.substr(4,2) + std::string("-") + iso_time_str.substr(6,2) + 
                        std::string("_") + iso_time_str.substr(9,2) + std::string("-") + iso_time_str.substr(11,2) + std::string("-") + iso_time_str.substr(13,2);
            std::string can_log_name = file_name_header + std::string(".CAN.log");
            std::string ros_bag_name = file_name_header + std::string(".ROS.bag");
            std::string logs_dir = std::string("~/do13_logs/");
            can_log_name = logs_dir + can_log_name;
            ros_bag_name = logs_dir + ros_bag_name;
            std::cout<<"ISO Time: "<<iso_time_str<<std::endl;
            std::cout<<"CAN log name: "<<can_log_name<<std::endl;
            std::cout<<"ROS bag name: "<<ros_bag_name<<std::endl;

            // Record CAN log for all CAN buses combined
            system(("candump any -ta >" + can_log_name + " &").c_str());
            // Record ROS bags for relevant topics only
            system(("rosbag record -e '(.*)gps(.*)' -e '(.*)pose(.*)' -e '(.*)cmd_vel(.*)' -e '(.*)/novatel/oem7(.*)' -e '(.*)local_planner_and_controller(.*)' -e '(.*)tf(.*)' -x '(.*)approach_object(.*)' -x '(.*)approach_object_behavior(.*)' -x '(.*)global_costmap(.*)' -e '(.*)global_planner(.*)' -x '(.*)goto_object_behavior(.*)' -x '(.*)local_costmap(.*)' -x '(.*)omnigraph(.*)' -x '(.*)point_cloud_pipeline(.*)' -x '(.*)point_cloud_cache(.*)' -e '(.*)local_planner(.*)' -e '(.*)odom(.*)' -e '(.*)navigation_manager(.*)' -O " + ros_bag_name + " __name:=rosbag_recording &").c_str());
            
            // Update recording state
            isRecording = true;
            ROS_INFO("Started data recording");
            ROS_INFO("Current System State = %d", msg->system_state);
        }
        else if (isRecording && (msg->system_state == SS6_READY || msg->system_state == SS31_SHUTDOWN_REQUESTED || msg->system_state >= 200))
        {
            // Wait for 10 seconds before killing data logging to capture crucial data in case of system error
            ros::Duration(10).sleep();

            // Kill rosbag record and CAN logging
            system("rosnode kill /rosbag_recording");
            system("killall -SIGKILL candump");

            // Update recording state
            isRecording = false;
            ROS_INFO("Rosbag record and CAN dump killed");
        }
    }
}
