#!/usr/bin/env bash

if [ ! -d build ]; then
    mkdir build
fi

cd build
cmake ..
make

mv libvelodyne_plugin.so ${GAZEBO_DEMO_LD} 
mv vel ${GAZEBO_DEMO_BIN}

