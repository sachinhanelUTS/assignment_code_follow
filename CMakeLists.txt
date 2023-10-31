cmake_minimum_required(VERSION 3.0.2)
project(assignment_code_follow)

find_package(OpenCV)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(gazebo REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  angles
  control_toolbox
  gazebo_ros
  robot_controllers
  robot_controllers_interface
  visp_camera_calibration
  visp_hand2eye_calibration
  visp_tracker
  visp_auto_tracker
  sensor_msgs
  roslib
  control_msgs
  geometry_msgs
  cv_bridge
  image_transport
  message_generation
  visp_bridge
)



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)


link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)



add_library(${PROJECT_NAME}-fetch_setup_temp src/fetch_setup_temp.cpp)


add_executable(${PROJECT_NAME}-follow_marker src/main.cpp src/follow_marker.cpp)


add_dependencies(${PROJECT_NAME}-follow_marker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME}-follow_marker ${catkin_LIBRARIES} ${PROJECT_NAME}-fetch_setup_temp ${OpenCV_LIBRARIES})

