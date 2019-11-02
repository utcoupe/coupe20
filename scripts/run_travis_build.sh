#!/bin/bash

CATKIN_CUSTOM_PARAMS=""

if [ "$TRAVIS_BRANCH" == "devel" ] || [ "$TRAVIS_BRANCH" == "master" ]; then
	echo "Adding tests to catkin."
	CATKIN_CUSTOM_PARAMS="-DUTCOUPE_RUN_TESTS:BOOL=1 $CATKIN_CUSTOM_PARAMS"
fi

COMPILE_COMMAND="catkin_make install $CATKIN_CUSTOM_PARAMS"

echo "--> final command : $COMPILE_COMMAND"

docker run \
    -i \
    --mount type=bind,source="$(pwd)"/ros_ws/src,target=/utcoupe/coupe20/ros_ws/src \
    --mount type=bind,source="$(pwd)"/libs,target=/utcoupe/coupe20/libs,readonly \
    utcoupe/coupe20:utcoupe-ros-kinetic-amd64 \
    /bin/bash -c "$COMPILE_COMMAND"
