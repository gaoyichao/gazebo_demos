cmake_minimum_required(VERSION 2.8.3)
project(gazebo_demos)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


catkin_package(

)

add_subdirectory(demos)

