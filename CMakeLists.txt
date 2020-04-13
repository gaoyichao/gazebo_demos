cmake_minimum_required(VERSION 2.8.3)
project(gazebo_demos)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  rospy
)

add_service_files(FILES SetPIDParams.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

add_subdirectory(demos)

