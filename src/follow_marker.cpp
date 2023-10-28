#include "../include/assignment_code_follow/follow_marker.h"





FollowMarker::FollowMarker(ros::NodeHandle node_handler)
    : node_handler_(node_handler)
{



  // Subscribe the node to the output topic of the aruco scanner 
  marker_subscriber_ = node_handler_.subscribe("/aruco_single/position", 1000, &FollowMarker::markerDetectedCallback, this);
  
  //odom_sub_ = node_handler_.subscribe("/odom", 100, &FollowMarker::odomCallBack, this);

  do_search_ = false;



  //set up the publisher for this node to output the data to modify the velocity of the fetch robot
  vel_pub_ = node_handler_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  duration_ = start_time_ - start_time_; // setting the time to 0 using the simulation time

  marker_.goal_distance_to_marker = 0.5 - 0.1076; //set max dist to 0.5, second number is the distance from the head of the robot to the base 
  ROS_INFO_STREAM("Tracking node initialised, will begin searching for aruco code");

}


FollowMarker::~FollowMarker()
{
}

//void FollowMarker::odomCallBack(const geometry_msgs::Vector3StampedPtr &msg)
//{ 
//}


void FollowMarker::markerDetectedCallback(const geometry_msgs::Vector3StampedPtr &msg)
{ 
  int marker_id;
  node_handler_.getParam("marker_id", marker_id);
  std::string marker_id_string = std::to_string(marker_id);

//Change frame of reference from robot to gazebo 
  marker_.pose.vector.x = msg->vector.z;//each one of these align
  marker_.pose.vector.y = msg->vector.x;
  marker_.pose.vector.z = msg->vector.y;
  ROS_INFO_STREAM("Located marker: " + marker_id_string + " ! Distance to marker: "
                "\n xVect: " + std::to_string(marker_.pose.vector.x) +
                "\n yVect: " + std::to_string(marker_.pose.vector.y) +
                "\n zVect: " + std::to_string(marker_.pose.vector.z));




  // as soon as a message is sent to the topic, the marker must be detected
  if (!marker_.detected)
  {
    marker_.detected = true;
    do_search_ = false;
    ROS_INFO_STREAM("code detected!");
  }

  double velocityScaled = marker_.absDist - marker_.goal_distance_to_marker; //proportional velocity 

  //ROS_INFO_STREAM(velocityScaled);

  //SETUP variables used to calculate the distance from the fetch to the marker 

  long xPosSquared = marker_.pose.vector.x*marker_.pose.vector.x;
  long yPosSquared = marker_.pose.vector.y*marker_.pose.vector.y;
  
  marker_.absDist = sqrt(xPosSquared + yPosSquared);





  if (marker_.absDist <= marker_.goal_distance_to_marker)
  {
    
    if (!marker_.reached)
    {
      ROS_INFO_STREAM("reached the marker!!!");
    }
    twistMsg_.linear.x = 0;
    twistMsg_.angular.z = 0;
  }

  //NOTES FOR TWIST COMMANDS:
  //positive x is forward
  //positive y is to the right of the robot if youre facing forward
  // Turn left or right
  if (marker_.pose.vector.y == 0)
    {
      twistMsg_.angular.z = 0;
    }
    else
    {
      twistMsg_.angular.z = -marker_.pose.vector.y;
    }


    // forwards/backwards movement
    if (marker_.absDist > marker_.goal_distance_to_marker)
    {
      twistMsg_.linear.x = velocityScaled / 2;
    }


    else if (marker_.absDist <= marker_.goal_distance_to_marker)
    {
      twistMsg_.linear.x = -0.3;
    }

  vel_pub_.publish(twistMsg_);

  // Start timing
  start_time_ = ros::Time::now();
}


//....

void FollowMarker::stop()
{
  while (ros::ok)
  {
    //reset duration timer
    if (duration_ > ros::Duration(10))
    {
      start_time_ = ros::Time::now();
    }

    duration_ = ros::Time::now() - start_time_;

    // Search for guider 
    if (duration_ >= ros::Duration(2.0) && duration_ <= ros::Duration(6.3) && !marker_.detected)
    {
      twistMsg_.angular.z = -1.43; 
      twistMsg_.linear.x = 0.0;
      vel_pub_.publish(twistMsg_);
      if (!do_search_)
      {
        ROS_INFO_STREAM("marker not found, searching for the marker");
        do_search_ = true;
      }
    }


    // Report no guider detected
    if (duration_ == ros::Duration(6.3) && !marker_.detected)
    {
      do_search_ = false;
      ROS_INFO_STREAM("No Guider detected. restarting search");
      start_time_ = ros::Time::now();
    }
  }

}