
#ifndef follow_marker_h
#define follow_marker_h

#include "ros/ros.h"

#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"//?
#include "tf2_msgs/TFMessage.h"
#include "fetch_lidar_output.h"
#include <cmath>
#include <chrono>
#include <vector>

class GuiderFollow
{
public:
    GuiderFollow(ros::NodeHandle node_handler);
    ~GuiderFollow();

    void markerCallback(const geometry_msgs::Vector3StampedPtr &msg);
    void laserCallBack(const sensor_msgs::LaserScanConstPtr &msg);
    void stop();

private:
    ros::NodeHandle node_handler_;

    ros::Subscriber marker_subscriber_;
    ros::Subscriber laser_sub_;
    ros::Publisher vel_pub_;
    geometry_msgs::Twist twistMsg_;
    ros::Subscriber pose_tracker_;

    LaserDetection laserDetection_;

    double laser_readings_;
    bool obstacle_detected_;
    bool obstacle_reported_;
    bool search_reported_;
    bool sweep_complete_;
    tf2_msgs::TFMessageConstPtr pose_fetch_;

    struct Marker
    {
        geometry_msgs::Vector3Stamped pose;
        double threshold_distance;
        const double head_to_base_offset = 0.1088; //taken from google from schematic of fetch robot
        double shortest_dist;
        bool detected;
        bool reached;
    };
    Marker marker_;

    ros::Time start_time_;   //!< start time
    ros::Duration duration_; //!< duration since start time (seconds)
};

#endif