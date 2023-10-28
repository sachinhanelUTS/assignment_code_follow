#!/usr/bin/env python

import rospy
import time
from nav_msgs.msg import Odometry  # Import Odometry from nav_msgs.msg
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

# Global variables for coordinates
x = 0.0
y = 0.0
theta = 0.0
detect = 0
startTimeAll = time.time()

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

sub = rospy.Subscriber("/turtleBurger/odom", Odometry, newOdom)
pub = rospy.Publisher("/turtleBurger/cmd_vel", Twist, queue_size = 1)
# Velocity command based on current position and orientation
# 1. If the robot is not facing the next position, rotate and face it
# 2. Once the robot is facing the position, move forward

speed = Twist()

r = rospy.Rate(50)  # 4Hz for publishing

goal = Point()
goal.x = 0
goal.y = 0
detect = 0
stage = 0
rotationDirection = 0

debugCounter = 0
debugMessage = "default"

angleDifStart = 0
angleDifEnd = 0
directionInvertCounter = 50

xGoals = [6.36, 3.23, 0.09, -4.92, -1.43, 6.93, 18.40] 

yGoals = [1.8, 5.42, 6.52, 2.12, -0.79, -4.6, -3.06]


while not rospy.is_shutdown():
    # Difference between the goal and the actual position of the robot
    endTimeAll = time.time()
    inc_x =  xGoals[stage]- x
    inc_y = yGoals[stage] - y
    # Now comparing the current and actual orientation arctan of the two differences in goal and current position
    angle_to_goal = atan2(inc_y, inc_x)
    

    
    #print("Stage:" + str(stage) +  "   X goal:" + str(xGoals[stage]) +  "   Y goal:" +  str(yGoals[stage]) +  "   Time Taken:" +  str(endTimeAll-startTimeAll) + "\n      XDist:" + str(inc_x) + "\n      YDist:" + str(inc_y) + "\n      XPos:" + str(x) + "\n      YPos:" + str(y)+ "\n      GoalAngDist:" + str(abs(angle_to_goal - theta)))

    debugCounter = debugCounter + 1

    if debugCounter == 1:
        angleDifStart = angle_to_goal - theta

    if debugCounter == 10:
        angleDifEnd = angle_to_goal - theta
        if (angleDifEnd - angleDifStart) < 0:
            rotationDirection = rotationDirection
        if (angleDifEnd - angleDifStart) > 0 and abs(angle_to_goal - theta) > 0.3:
            directionInvertCounter = directionInvertCounter + 10
            print("increasing...")
        if (angleDifEnd - angleDifStart) > 0 and directionInvertCounter == 110:
            rotationDirection = not rotationDirection
            directionInvertCounter = 0
        print("Stage:" + str(stage) +  "   X goal:" + str(xGoals[stage]) +  "   Y goal:" +  str(yGoals[stage]) +  "   Time Taken:" 
              +  str(endTimeAll-startTimeAll) + "   Rotation:" +  str(rotationDirection) +  " InvCounter:" + str(directionInvertCounter) + "\n      XDist:" 
              + str(inc_x) + "\n      YDist:" + str(inc_y) + "\n      XPos:" + str(x) 
              + "\n      YPos:" + str(y)+ "\n      GoalAngDist:" + str(abs(angle_to_goal - theta)))
        print(debugMessage)
        debugCounter = 0


    if abs(angle_to_goal - theta) > 1 and rotationDirection == 1:
        speed.linear.x = 0.0
        speed.angular.z = 0.55
        debugMessage = "Locating goal"

    if abs(angle_to_goal - theta) > 1 and rotationDirection == 0:
        speed.linear.x = 0.0
        speed.angular.z = -0.55
        debugMessage = "Locating goal"

    # To align the angles, if not aligned it will rotate only FOR MORE FINER ADJUSTMENT
    if abs(angle_to_goal - theta) > 0.3 and abs(angle_to_goal - theta) < 1 and  rotationDirection == 1:
        speed.linear.x = 0.0
        speed.angular.z = 0.15
        debugMessage = "Locating goal"

    if abs(angle_to_goal - theta) > 0.3 and abs(angle_to_goal - theta) < 1 and rotationDirection == 0:
        speed.linear.x = 0.0
        speed.angular.z = -0.15
        debugMessage = "Locating goal"

    # If facing the goal, then move to it
    if abs(angle_to_goal - theta) < 0.3:
        speed.linear.x = 0.5
        speed.angular.z = 0.0
        debugMessage = "Moving to goal"
        detect = 1
        directionInvertCounter = 0
    
    if abs(inc_x) < 0.1 and abs(inc_y) < 0.1:
        debugMessage = "goal reached !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        stage = stage + 1
        goal.x = xGoals[stage]
        goal.y = yGoals[stage]

    

    # Now publish this value via the publisher
    pub.publish(speed)
    r.sleep()
