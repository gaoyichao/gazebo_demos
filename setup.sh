#!/usr/bin/env bash

WS_PATH=`pwd`

export GAZEBO_DEMO_PATH=${WS_PATH}
export GAZEBO_DEMO_LD=${WS_PATH}/lib
export GAZEBO_DEMO_BIN=${WS_PATH}/bin

export GAZEBO_MODEL_PATH=${WS_PATH}/models:$GAZEBO_MODEL_PATH


