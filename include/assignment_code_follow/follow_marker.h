
#ifndef follow_marker_h
#define follow_marker_h

#include <cmath>
#include <chrono>
#include <vector>

#include "ros/ros.h"

#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Twist.h"
#include "tf2_msgs/TFMessage.h"




class FollowMarker
{
public:
    FollowMarker(ros::NodeHandle node_handler);
    ~FollowMarker();

    void markerCall(const geometry_msgs::Vector3StampedPtr &msg);
    void odomCallBack(const geometry_msgs::Vector3StampedPtr &msg);
    void stop();

private:


    struct Marker
    {
        geometry_msgs::Vector3Stamped pose;
        bool reached;
        double goal_distance_to_marker;
        long absDist;
        bool detected;

    };
    Marker marker_;

    geometry_msgs::Twist twist_msg;


    ros::NodeHandle node_handler_;

    ros::Subscriber marker_subscriber;
    ros::Publisher vel_publisher;
    
    ros::Time start_time_;   
    ros::Duration duration_; 


    long yPosSquared;
    long xPosSquared;

    bool publishSearchMsg;

    int marker_id;


    tf2_msgs::TFMessageConstPtr pose_fetch_;

    
    
};

#endif