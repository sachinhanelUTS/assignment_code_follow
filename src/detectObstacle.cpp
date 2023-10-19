#include "../include/assignment_code_follow/detectObstacle.h"

DetectObstacle::DetectObstacle(ros::NodeHandle node_handler) : node_handler_(node_handler)
{
    sub1_ = node_handler_.subscribe("/base_scan_raw", 10, &DetectObstacle::laserCallBack, this);
}

void DetectObstacle::state()
{
    while (ros::ok())
    {
        if (path_obstructed_ == false)
        {
            ROS_INFO_STREAM("An obstacle has been detected!!!");
        }
        if (path_obstructed_== true)
        {
            ROS_INFO_STREAM("Moving!!!");
        }
    }
}

void DetectObstacle::laserCallBack(const sensor_msgs::LaserScanConstPtr &msg)
{
    path_obstructed_= laserDetection_.obstructionDetect(msg);
    laser_readings_ = laserDetection_.laserReadings(msg);
}