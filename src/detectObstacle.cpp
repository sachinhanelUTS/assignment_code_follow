#include "../include/assignment_code_follow/detectObstacle.h"

EnvironmentSensing::EnvironmentSensing(ros::NodeHandle node_handler) : node_handler_(node_handler)
{
    sub1_ = node_handler_.subscribe("/base_scan_raw", 10, &EnvironmentSensing::laserCallBack, this);
}

void EnvironmentSensing::status()
{
    while (ros::ok())
    {
        if (obstacle_detected_ == false)
        {
            ROS_INFO_STREAM("Detected  wall");
        }
        else
        {
            ROS_INFO_STREAM("Move");
        }
    }
}

void EnvironmentSensing::laserCallBack(const sensor_msgs::LaserScanConstPtr &msg)
{

    obstacle_detected_ = laserDetection_.detectObtacle(msg);
    laser_readings_ = laserDetection_.getLaserReading(msg);
    //ROS_INFO_STREAM(laser_readings_);
}