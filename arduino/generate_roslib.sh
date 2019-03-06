#!/bin/bash

ARDUINO_LIBS_DIR="$HOME/sketchbook/libraries"

function clean_arduino_lib() {
    if [ -d "$ARDUINO_LIBS_DIR/ros_lib" ]; then
        "Cleaning old ros_lib for Arduino"
        rm -rf "$ARDUINO_LIBS_DIR/ros_lib"
    fi
}

function generate_roslib() {
    pushd .
    cd "$UTCOUPE_WORKSPACE/ros_ws"
    rm -rf build devel
    catkin_make
    source devel/setup.bash
    if [ ! -d "$ARDUINO_LIBS_DIR" ]; then
        echo "Warning: is the Arduino IDE installed?"
        mkdir -p "$ARDUINO_LIBS_DIR"
    fi
    cd "$ARDUINO_LIBS_DIR"
    rosrun rosserial_arduino make_libraries.py .
    echo "Succeeded to generate ros_lib for Arduino"
    popd
}

clean_arduino_lib
generate_roslib
