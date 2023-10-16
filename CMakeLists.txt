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
  sensor_msgs
  roslib
  control_msgs
  geometry_msgs
  cv_bridge
  image_transport
  message_generation
  visp_auto_tracker
  visp_bridge
  visp_camera_calibration
  visp_hand2eye_calibration
  visp_tracker
)



include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


catkin_package(
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



add_library(${PROJECT_NAME}-laserScan src/laserScan.cpp)

add_executable(${PROJECT_NAME}-marker_turtle_teleop src/marker_turtle_teleop.cpp)
add_executable(${PROJECT_NAME}-follow_marker src/main.cpp src/follow_marker.cpp)

add_dependencies(${PROJECT_NAME}-marker_turtle_teleop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}-follow_marker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}-marker_turtle_teleop ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}-follow_marker ${catkin_LIBRARIES} ${PROJECT_NAME}-laserScan ${OpenCV_LIBRARIES})

