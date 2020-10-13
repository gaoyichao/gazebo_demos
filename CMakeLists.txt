cmake_minimum_required(VERSION 2.8.3)
project(gazebo_demos)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

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

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# 
add_service_files(FILES SetPIDParams.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

set(PROTO_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/proto)
include_directories(${PROTO_GEN_DIR})
link_directories(${PROTO_GEN_DIR})
add_subdirectory(proto)

add_subdirectory(demos)

add_library(gazebo_demos_ros_plugin src/ros_plugin.cc)
target_link_libraries(gazebo_demos_ros_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})




