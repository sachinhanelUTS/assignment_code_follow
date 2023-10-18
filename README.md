# assignment_code_follow
![image](https://github.com/sachinhanelUTS/assignment_code_follow/assets/64720029/b5736388-f124-4397-8116-23efcda29aa4)

## What the Code does
The following code should allow a fetch robot to follow and track a marker through a custom environment. The fetch robot will rotate and scan for the correct marker using the Aruco ROS library detector.
Once the marker is spotted the robot will follow at a 0.5m distance, if there is an obstacle the robot should avoid it. The marker is attached to a turtle bot that can be controlled. </br>

## Contributors
Arya Nooralizadeh </br>
- Files and ROS research </br>
- Src files </br>
- Launch files </br>

Sachin Hanel </br>
- Files and ROS research </br>
- Src files </br>
- Launch files </br>

James Farrel </br>
- Files and ROS research </br>
- Environment creation </br>
- Custom models </br>

## Startup requirements
- The models need to be moved from the models folder to the ROS directory folder models. </br>
- Also please ensure that the png. has been correctly moved to the .gazebo >>> custom_turtlebot >>> meshes >>> textures. </br>
If this step is not done the fetch bot will ignore the marker if any are visible. </br>

## Dependencies
The program uses the Fetch robot and the Turtle bot packages. Please ensure that they are installed in the catkin workspace. 

## Aruco Marker Detection
This is used to detect and track the marker on the turtle bot </br>
<code>
cd ~/catkin_ws/src </br>
git clone https://github.com/pal-robotics/aruco_ros.git </br>
cd ~/catkin_ws </br>
catkin_make </br>
</code>

## Installing this package
To install this package, navigate to the green "code" button and press it, there should be an option to copy the HTTPS link </br>
<code>
cd ~/catkin_ws/src </br>
git clone **Insert_Link_here** </br>
cd ~/catkin_ws </br>
catkin_make
</code>

# Operation
**Initial environment** </br>
</br>
<code>roslaunch assignment_code_follow spawn_fetch_setup.launch</code> </br>
   
**Begin tracking** </br>
</br>
<code>roslaunch assignment_code_follow marker_follow.launch </code></br>
</br>**Note:** Do not delete the RQT tab that pops up </br>

**Control the burger turtle bot marker** </br>
</br>
3. <code>rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/robot1/cmd_vel</code> </br>

**RViz** </br>
</br>
4. <code>rosrun rviz rviz -d ~/Documents/catkin_ws/src/assignment_code_follow/rviz/rviz_fetch_debug.rviz
</code> </br></br>
**Note:** Open Rviz >>> Open config >>> Navigate to this folder >>> Open Rviz folder</br>

## **Acknowledgments**
The following resources were used to aid in the making of this project: </br>
<ul>
  <li>https://github.com/ZebraDevs/fetch_ros</li>
  <li>https://github.com/fetchrobotics/docs</li>
  <li>https://www.youtube.com/watch?v=u113ctJtd3M&ab_channel=LeTrung</li>
  <li>https://github.com/ros-planning/navigation_tutorials</li>
	<li>https://wiki.ros.org/navigation_tutorials</li>
	<li>https://github.com/esteban-andrade/FetchRobotPathFollow</li>
	<li>https://github.com/TRECVT/visp_auto_tracker</li>
	<li>https://github.com/pal-robotics/aruco_ros</li>
	<li>9 item</li>
</ul>

