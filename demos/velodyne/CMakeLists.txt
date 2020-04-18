cmake_minimum_required(VERSION 2.8.3)

# Find PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(PythonLibs 2.7)
include_directories(${PYTHON_INCLUDE_DIRS})

# Build our plugin
add_library(velodyne_plugin SHARED velodyne_plugin.cc)
target_link_libraries(velodyne_plugin gazebo_demos_proto ${GAZEBO_libraries} ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
add_dependencies(velodyne_plugin gazebo_demos_proto)

# Build the stand-alone test program
add_executable(vel vel.cc)
target_link_libraries(vel gazebo_demos_proto ${GAZEBO_LIBRARIES})
add_dependencies(vel gazebo_demos_proto)


