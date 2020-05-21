#!/usr/bin/env bash

WS_PATH=`pwd`

export GAZEBO_DEMO_PATH=${WS_PATH}
export GAZEBO_DEMO_LD=${WS_PATH}/lib
export GAZEBO_DEMO_BIN=${WS_PATH}/bin

export LD_LIBRARY_PATH=${GAZEBO_DEMO_LD}:/opt/ros/melodic/lib:${LD_LIBRARY_PATH}
export PATH=${GAZEBO_DEMO_BIN}:${PATH}
export GAZEBO_MODEL_PATH=${WS_PATH}/models:$GAZEBO_MODEL_PATH

