#!/bin/bash

CATKIN_CUSTOM_PARAMS="-DCMAKE_BUILD_TYPE=Release"

if [ "$TRAVIS_BRANCH" == "devel"] || [ "$TRAVIS_BRANCH" == "master" ]; then
    CATKIN_CUSTOM_PARAMS="-DUTCOUPE_BUILD_TESTS=true $CATKIN_CUSTOM_PARAMS"
fi

docker run \
    -i \
    --mount type=bind,source="$(pwd)"/ros_ws/src,target=/utcoupe/coupe19/ros_ws/src \
    --mount type=bind,source="$(pwd)"/libs,target=/utcoupe/coupe19/libs,readonly \
    utcoupe/coupe19:utcoupe-ros-kinetic-amd64 \
    /bin/bash -c "catkin_make install $CATKIN_CUSTOM_PARAMS"
