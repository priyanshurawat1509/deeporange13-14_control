#include "ros/ros.h"
// #include "rosgraph_msgs/Log.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_msgs/TFMessage.h"
#include "novatel_oem7_msgs/INSPVAX.h"
#include <deeporange13_msgs/RosHealth.h>
#include <string>
#include <ros/console.h>
#include "geometry_msgs/Twist.h"
#include <deeporange13_control/state_enums.h>


namespace deeporange_dbw_ros
{

class RosHealthMonitor
{
    public:
    //constructor
    RosHealthMonitor(ros::NodeHandle& node, ros::NodeHandle &priv_nh);

    //destructor
    ~RosHealthMonitor();

    private:
    //member-variables
    std::string consoleMsg;
    int solnType = 0;
    int sevLvl = 0;
    int rtk_checkStart_1 = 0;
    int rtk_checkStart_2 = 0;
    double startTimer = 0;
    float lidar_tSec = 0;
    float tf_tSec = 0;
    int rtk_check = 0;
    std::string do_ns;

    //member functions
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg);
    void inspvaxCallback(const novatel_oem7_msgs::INSPVAX::ConstPtr& msg);
    void publishROSHealth(const ros::TimerEvent& event);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    
    //publishers and subscribers
    // ros::Subscriber rosout_sub;
    ros::Subscriber rtk_sub;
    ros::Publisher health_pub;
    ros::Subscriber tf_sub;
    ros::Subscriber lidar_sub;
    ros::Subscriber sub_cmdVel_;
    
    ros::Timer timer;
    ros::Time phxLastCmdVelReceiveTime_;

    //member messages
    deeporange13_msgs::RosHealth healthMsg;

};

}
