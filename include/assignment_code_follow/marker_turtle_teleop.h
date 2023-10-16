
#ifndef GUIDER_JOY_TELEOP_H
#define GUIDER_JOY_TELEOP_H

#include "ros/ros.h"

#include <thread>

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32.h"


class GuiderJoyTeleop
{
public:

    GuiderJoyTeleop(ros::NodeHandle node_handler);
    ~GuiderJoyTeleop();

    void joyCallback(const sensor_msgs::JoyPtr &msg);

    void velocityPublisher();

private:
    ros::NodeHandle node_handler_;  
    
    ros::Subscriber joy_sub_;
    ros::Publisher vel_pub_;
    geometry_msgs::Twist twistMsg_; 
};


#endif // GUIDER_JOY_TELEOP_H