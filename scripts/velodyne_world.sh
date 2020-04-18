#!/bin/sh
[ -L ${0} ] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR=$(dirname ${SCRIPT_DIR})
WORLD_DIR=${SCRIPT_DIR}/../worlds 

gazebo ${WORLD_DIR}/velodyne.world -s `catkin_find --first-only libgazebo_demos_ros_plugin.so`

