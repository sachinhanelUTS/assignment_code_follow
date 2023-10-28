#include "ros/ros.h"
#include "../include/assignment_code_follow/follow_marker.h"

#include <thread>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "guider_follow");

    ros::NodeHandle node_handler;

    std::shared_ptr<FollowMarker> robot(new FollowMarker(node_handler));
    std::thread vel(&FollowMarker::stop, robot);
    ros::spin();

    ros::shutdown();

    vel.join();

    return 0;
}