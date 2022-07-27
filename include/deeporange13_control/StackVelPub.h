#include <string.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <can_msgs/Frame.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>

class VehicleMotionPublisher
{
    public:
    VehicleMotionPublisher(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
    ~VehicleMotionPublisher();
    double xvel_;
    double angularvel_;
    ros::Publisher pub_cmdVel_;
    ros::Subscriber sub_switch_;
    geometry_msgs::Twist twistMsg_;
    int switch_;
    
    private:
    void publishVel(const ros::TimerEvent& event);
    void callback(std_msgs::Bool msg);
    ros::Timer timer_;

};
