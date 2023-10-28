
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

    void markerDetectedCallback(const geometry_msgs::Vector3StampedPtr &msg);
    void odomCallBack(const geometry_msgs::Vector3StampedPtr &msg);
    void stop();

private:


    struct Marker
    {
        geometry_msgs::Vector3Stamped pose;
        double goal_distance_to_marker;
        double shortest_dist;
        long absDist;


        bool detected;
        bool reached;
    };
    Marker marker_;

    geometry_msgs::Twist twistMsg_;
    ros::Subscriber pose_tracker_;

    ros::NodeHandle node_handler_;

    ros::Subscriber marker_subscriber_;
    ros::Publisher vel_pub_;
    
    ros::Time start_time_;   
    ros::Duration duration_; 


    bool obstacle_reported_;
    long yPosSquared;
    long xPosSquared;

    bool do_search_;
    
    int marker_id;


    tf2_msgs::TFMessageConstPtr pose_fetch_;

    
    
};

#endif