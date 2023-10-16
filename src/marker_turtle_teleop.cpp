#include "../include/assignment_code_follow/marker_turtle_teleop.h"

GuiderJoyTeleop::GuiderJoyTeleop(ros::NodeHandle node_handler)
    :node_handler_(node_handler)
{
    joy_sub_ = node_handler_.subscribe("/joy", 1000, &GuiderJoyTeleop::joyCallback, this);

    vel_pub_ = node_handler_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
}

GuiderJoyTeleop::~GuiderJoyTeleop(){

}

void GuiderJoyTeleop::joyCallback(const sensor_msgs::JoyPtr &msg){
    
    twistMsg_.linear.x = msg->axes.at(1)*2;
    twistMsg_.angular.z = msg->axes.at(3)*2;
}

void GuiderJoyTeleop::velocityPublisher(){
    ros::Rate loop_rate(20);
    while (ros::ok()){
        vel_pub_.publish(twistMsg_);
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_ps4_controller");

  ros::NodeHandle node_handler;
  
  std::shared_ptr<GuiderJoyTeleop> robot(new GuiderJoyTeleop(node_handler));
  std::thread vel(&GuiderJoyTeleop::velocityPublisher,robot);

  ros::spin();

  ros::shutdown();
  vel.join();

  return 0;
}