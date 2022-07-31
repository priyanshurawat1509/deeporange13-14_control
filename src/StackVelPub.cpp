/*
To simulate stack cmd_vels for testing interface pipeline with raptor
*/

#include <deeporange13_control/StackVelPub.h>

VehicleMotionPublisher::VehicleMotionPublisher(ros::NodeHandle &node, ros::NodeHandle &priv_nh)
{   
    sub_switch_ = node.subscribe("/cmd_velocity_choice", 10, &VehicleMotionPublisher::choiceCallback, this);
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

void VehicleMotionPublisher::choiceCallback(std_msgs::Int8 msg)
{
    switch_ = msg.data; 
}

void VehicleMotionPublisher::publishVel(const ros::TimerEvent& event)
/*
0 - zero
1 -linear.x only
2 - angular.z - left turn only
3 - angular.z - right turn only
4 - linear and angular left turn 
5 - fast linear.x only
6 - fast angular.z - left turn only
7 - fast angular.z - right turn only
8 - fast linear and angular left turn 

*/
{
    if (switch_ == 0)
    {
        twistMsg_.linear.x = 0;
        twistMsg_.angular.z = 0;
    }
    else if (switch_ == 1)
    {
        twistMsg_.linear.x = 0.8; //1.78 mph
        twistMsg_.angular.z = 0.0;  
    }
    else if (switch_ == 2)
    {
        twistMsg_.linear.x = 0.0;
        twistMsg_.angular.z = 1.78;  // 4 mph
    }  
    else if (switch_ == 3)
    {
        twistMsg_.linear.x = 0.0;
        twistMsg_.angular.z = -1.78;  // 4 mph
    }   
    else if (switch_ == 4)
    {
        twistMsg_.linear.x = 1;  // 2.237 mph
        twistMsg_.angular.z = 0.4;  //0.89 mph
    }   
    else if (switch_ == 5)
    {
        twistMsg_.linear.x = 1.8;  //4.02 mph
        twistMsg_.angular.z = 0.0;  
    }     
    else if (switch_ == 6)
    {
        twistMsg_.linear.x = 0.0;
        twistMsg_.angular.z = 2.1;  // 5 mph
    }   
    else if (switch_ == 7)
    {
        twistMsg_.linear.x = 0.0;
        twistMsg_.angular.z = -2.1;  // 5.14mph
    }   
    else if (switch_ == 8)
    {
        twistMsg_.linear.x = 1.8;
        twistMsg_.angular.z = 0.8;  
    }
    else
    {
        twistMsg_.linear.x = 0.0;
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
