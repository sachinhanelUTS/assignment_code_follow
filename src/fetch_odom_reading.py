#!/usr/bin/env python
import rospy
import time
import tf
from nav_msgs.msg import Odometry  # Import Odometry from nav_msgs.msg
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2


class Odometry_data:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


rospy.init_node('fetch_odom_reading', anonymous=True)

marker_publisher = rospy.Publisher('box_visual',Odometry, queue_size=100)
rospy.sleep(1)

odom = Odometry_data(0.0, 0.0, 0.0)

def get_odom(msg):
    print("inside the get_odom\n")
    # print msg.pose.pose
    x_odom = msg.pose.pose.position.x
    y_odom = msg.pose.pose.position.y
    quat_x = msg.pose.pose.orientation.x
    quat_y = msg.pose.pose.orientation.y
    quat_z = msg.pose.pose.orientation.z
    quat_w = msg.pose.pose.orientation.w
    euler = tf.transformations.euler_from_quaternion([quat_x, quat_y, quat_z, quat_w])
    # print "euler = ", euler
    theta_odom = euler[2]
    # print theta_odom

    # Transform the odometry data wrt the hokuyo_link frame
    x_odom_hokuyo = x_odom + 0.377
    global odom
    odom = Odometry_data(x_odom_hokuyo, y_odom, -theta_odom)
    print("odom_2 = " + odom.x + " " + odom.y, " " +  odom.theta + "\n")

#def callback(data):
    
    
    

def face_finder():

    rospy.Subscriber("/odom", Odometry, get_odom)
    print("odom_2 = " + odom.x + " ", odom.y + " " + odom.theta + "\n")

    rospy.spin()

if __name__ == '__main__':
    face_finder()
