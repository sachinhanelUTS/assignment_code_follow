#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry  # Import Odometry from nav_msgs.msg
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

# Global variables for coordinates
x = 0.0
y = 0.0
theta = 0.0
detect = 0

def newOdom(msg):
    global x
    global y
    global theta
    global detect

    # Where in the Odometry the location msg is
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    # Cant obtain as above since quaternion so need eulers, gives 3 different angles
    rot_q = msg.pose.pose.orientation  # Getting the quaternion
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])  # Getting each component of quaternion

rospy.init_node("speed_controller")

sub = rospy.Subscriber("/robot1/odom", Odometry, newOdom)
pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size = 1)
# Velocity command based on current position and orientation
# 1. If the robot is not facing the next position, rotate and face it
# 2. Once the robot is facing the position, move forward

speed = Twist()

r = rospy.Rate(4)  # 4Hz for publishing

goal = Point()
goal.x = 0
goal.y = 0
detect = 0

while not rospy.is_shutdown():
    # Difference between the goal and the actual position of the robot
    inc_x = goal.x - x
    inc_y = goal.y - y

    # Now comparing the current and actual orientation arctan of the two differences in goal and current position
    angle_to_goal = atan2(inc_y, inc_x)

    # To align the angles, if not aligned it will rotate only
    if abs(angle_to_goal - theta) > 0.3 and detect == 0:
        speed.linear.x = 0.0
        speed.angular.z = 0.3
        print("Locating goal")

    if abs(angle_to_goal - theta) > 0.3 and detect == 1:
        speed.linear.x = 0.0
        speed.angular.z = -0.3
        print("Locating goal")

    # If facing the goal, then move to it
    else:
        speed.linear.x = 0.5
        speed.angular.z = 0.0
        print("moving to goal")
        detect = 1

        

    # Now publish this value via the publisher
    pub.publish(speed)
    r.sleep()
