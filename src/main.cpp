#include "ros/ros.h"
#include "../include/assignment_code_follow/follow_marker.h"

#include <thread>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "guider_follow");

    ros::NodeHandle nh;

    std::shared_ptr<GuiderFollow> robot(new GuiderFollow(nh));
    std::thread vel(&GuiderFollow::stop, robot);
    ros::spin();

    ros::shutdown();

    vel.join();

    return 0;
}