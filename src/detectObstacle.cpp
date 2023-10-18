#include "../include/assignment_code_follow/detectObstacle.h"

DetectObstacle::DetectObstacle(ros::NodeHandle node_handler) : node_handler_(node_handler)
{
    sub1_ = node_handler_.subscribe("/base_scan_raw", 10, &DetectObstacle::laserCallBack, this);
}

void DetectObstacle::state()
{
    while (ros::ok())
    {
        if (obstacle_detected_ == false)
        {
            ROS_INFO_STREAM("An obstacle has been detected!!!");
        }
        if (obstacle_detected_ == true)
        {
            ROS_INFO_STREAM("Moving!!!");
        }
    }
}

void DetectObstacle::laserCallBack(const sensor_msgs::LaserScanConstPtr &msg)
{
    obstacle_detected_ = laserDetection_.detectObtacle(msg);
    laser_readings_ = laserDetection_.getLaserReading(msg);
}