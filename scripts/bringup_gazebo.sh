#!/bin/sh
[ -L ${0} ] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR=$(dirname ${SCRIPT_DIR})

export GAZEBO_DEMO_PATH=$(dirname ${SCRIPT_DIR})
export GAZEBO_MODEL_PATH=${GAZEBO_DEMO_PATH}/models:$GAZEBO_MODEL_PATH

if [ -f ${1} ]
then
    gazebo ${1} -s `catkin_find --first-only libgazebo_demos_ros_plugin.so`
fi

