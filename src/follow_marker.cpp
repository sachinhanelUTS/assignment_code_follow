#include "../include/assignment_code_follow/follow_marker.h"





FollowMarker::FollowMarker(ros::NodeHandle node_handler)
    : node_handler_(node_handler)
{
  ROS_INFO_STREAM("Tracking node initialised, will begin searching for aruco code");


  // Subscribe the node to the output topic of the aruco scanner 
  marker_subscriber = node_handler_.subscribe("/aruco_single/position", 1000, &FollowMarker::markerCall, this);
  


  //odom_sub_ = node_handler_.subscribe("/odom", 100, &FollowMarker::odomCallBack, this);

  marker_.goal_distance_to_marker = 1 - 0.1076; //set max dist to 0.5, second number is the distance from the head of the robot to the base 
  
  publishSearchMsg = false;



  //set up the publisher for this node to output the data to modify the velocity of the fetch robot
  vel_publisher = node_handler_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  duration_ = start_time_ - start_time_; // setting the time to 0 using the simulation time

  

}


FollowMarker::~FollowMarker()
{
}

//void FollowMarker::odomCallBack(const geometry_msgs::Vector3StampedPtr &msg)
//{ 
//}



void FollowMarker::stop() //    what to do when the marker is no longer detected, i.e begin searching and what to do if its not found after this 
{
  while (ros::ok)
  {
    duration_ = ros::Time::now() - start_time_;

    // search clockwise for marker
    if (duration_ >= ros::Duration(2.0) && duration_ <= ros::Duration(6.3) && !marker_.detected)
    {
      twist_msg.angular.z = -1.43; 
      twist_msg.linear.x = 0.0;
      vel_publisher.publish(twist_msg);
      if (!publishSearchMsg)
      {
        ROS_INFO_STREAM("marker not found, searching for the marker");
        publishSearchMsg = true;
      }
    }


    // Report no guider detected
    if (duration_ == ros::Duration(6.3) && !marker_.detected)
    {
      publishSearchMsg = false;
      ROS_INFO_STREAM("No Guider detected. restarting search");
      start_time_ = ros::Time::now();
    }


    if (duration_ > ros::Duration(10))
      {
      start_time_ = ros::Time::now();
      }
  }
}

   


void FollowMarker::markerCall(const geometry_msgs::Vector3StampedPtr &msg)
{ 
  int marker_id;
  node_handler_.getParam("marker_id", marker_id);
  std::string marker_id_string = std::to_string(marker_id);

//Change frame of reference from robot to gazebo 
  marker_.pose.vector.x = msg->vector.z;
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
    publishSearchMsg = false;
    ROS_INFO_STREAM("code detected!");
  }

  double velocityScaled = marker_.absDist - marker_.goal_distance_to_marker; //proportional control velocity calculated 

  //ROS_INFO_STREAM(velocityScaled);

  //SETUP variables used to calculate the distance from the fetch to the marker 

  long xPosSquared = marker_.pose.vector.x*marker_.pose.vector.x;
  long yPosSquared = marker_.pose.vector.y*marker_.pose.vector.y;
  
  marker_.absDist = sqrt(xPosSquared + yPosSquared);





  if (marker_.absDist <= marker_.goal_distance_to_marker) //detect if the marker is within the requried distance
  {
    
    if (!marker_.reached)
    {
      ROS_INFO_STREAM("reached the marker!!!");
    }
    twist_msg.linear.x = 0;
    twist_msg.angular.z = 0;
  }

  //NOTES FOR TWIST COMMANDS:
  //positive x is forward
  //positive y is to the right of the robot if youre facing forward
  // Turn left or right
  if (marker_.pose.vector.y == 0)
    {
      twist_msg.angular.z = 0;
    }
    else
    {
      twist_msg.angular.z = -marker_.pose.vector.y;
    }


    // forwards/backwards movement
    if (marker_.absDist > marker_.goal_distance_to_marker)
    {
      twist_msg.linear.x = velocityScaled / 1.5;
    }


    else if (marker_.absDist <= marker_.goal_distance_to_marker)
    {
      twist_msg.linear.x = -0.3;
    }

  vel_publisher.publish(twist_msg);

  // Start timing
  start_time_ = ros::Time::now();
}
