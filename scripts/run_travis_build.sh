#!/bin/bash

CATKIN_CUSTOM_PARAMS=""

if [ "$TRAVIS_BRANCH" == "devel" ] || [ "$TRAVIS_BRANCH" == "master" ]; then
	echo "Adding tests to catkin."
	CATKIN_CUSTOM_PARAMS="-DUTCOUPE_BUILD_TESTS:BOOL=ON $CATKIN_CUSTOM_PARAMS"
fi

COMPILE_COMMAND="catkin_make install $CATKIN_CUSTOM_PARAMS"

echo "--> final command : $COMPILE_COMMAND"

docker run \
    -i \
    --mount type=bind,source="$(pwd)"/ros_ws/src,target=/utcoupe/coupe19/ros_ws/src \
    --mount type=bind,source="$(pwd)"/libs,target=/utcoupe/coupe19/libs,readonly \
    utcoupe/coupe19:utcoupe-ros-kinetic-amd64 \
    /bin/bash -c "$COMPILE_COMMAND"
