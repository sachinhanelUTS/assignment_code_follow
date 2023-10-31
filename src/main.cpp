#include "ros/ros.h"
#include "../include/assignment_code_follow/follow_marker.h"

#include <cmath>
#include <chrono>
#include <vector>

#include "ros/ros.h"

#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Twist.h"
#include "tf2_msgs/TFMessage.h"

#include <thread>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker_follow");
    ros::NodeHandle node_handler;

    std::shared_ptr<FollowMarker> robot(new FollowMarker(node_handler)); //creating a followmarker instance to represent the robot
    std::thread fetchFollow(&FollowMarker::stop, robot); //creating the thread and 
    
    ros::spin();//enter ros processing loop
    ros::shutdown();
    fetchFollow.join();

    return 0;
}