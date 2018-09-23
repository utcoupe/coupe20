#!/bin/bash

docker run \
    -i \
    --mount type=bind,source="$(pwd)"/ros_ws/src,target=/utcoupe/coupe19/ros_ws/src,readonly \
    --mount type=bind,source="$(pwd)"/libs,target=/utcoupe/coupe19/libs,readonly \
    utcoupe/coupe19:utcoupe-ros-kinetic-amd64 \
    /bin/bash -c "catkin_make install -DCMAKE_BUILD_TYPE=Release"