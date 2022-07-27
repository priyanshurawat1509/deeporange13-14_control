#include <deeporange13_control/StackVelPub.h>

VehicleMotionPublisher::VehicleMotionPublisher(ros::NodeHandle &node, ros::NodeHandle &priv_nh)
{   
    sub_switch_ = node.subscribe("/switch", 10, &VehicleMotionPublisher::callback, this);
    pub_cmdVel_ = node.advertise<geometry_msgs::Twist>("/deeporange1314/cmd_vel", 10);
    node.getParam("xvel", xvel_);
    node.getParam("angularvel", angularvel_);
    twistMsg_.linear.x = 0;
    twistMsg_.linear.y = 0;
    twistMsg_.linear.z = 0;
    twistMsg_.angular.x = 0;
    twistMsg_.angular.y = 0;
    twistMsg_.angular.z = 0;
    switch_ = 0;
    
    timer_ = node.createTimer(ros::Duration(1.0 / 20.0), &VehicleMotionPublisher::publishVel, this);
};

VehicleMotionPublisher::~VehicleMotionPublisher(){};

void VehicleMotionPublisher::callback(std_msgs::Bool msg)
{
    if (msg.data == true)
    {
        switch_ = 1;
    }
    else
    {
        switch_ = 0; // zeros
    }

}
void VehicleMotionPublisher::publishVel(const ros::TimerEvent& event)
{
    if (switch_ == 0)
    {
        twistMsg_.linear.x = 0;
        twistMsg_.angular.z = 0;
    }
    else
    {
        twistMsg_.linear.x = 0.8;
        twistMsg_.angular.z = 0.0;  
    }

    pub_cmdVel_.publish(twistMsg_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "VehicleMotionPublisher");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");
  
  VehicleMotionPublisher n_velPub(node, priv_nh);

  
  
  // handle callbacks until shut down
  ros::spin();

  return 0;
}
