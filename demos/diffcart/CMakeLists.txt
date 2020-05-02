cmake_minimum_required(VERSION 2.8.3)

# Find PCL
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(PythonLibs 2.7)
include_directories(${PYTHON_INCLUDE_DIRS})

# Build our plugin
add_library(diffcart_plugin SHARED diffcart_plugin.cc)
target_link_libraries(diffcart_plugin gazebo_demos_proto ${GAZEBO_libraries} ${catkin_LIBRARIES} ${PYTHON_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(diffcart_plugin gazebo_demos_proto)


