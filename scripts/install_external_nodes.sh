#!/bin/bash

# The goal of this script is to compile and install external ROS nodes used by the UTCoupe nodes. This avoids to compile the external nodes each time the UTCoupe workspace is recompiled.

ARCH=$(uname -m)
INSTALL_ROOT=/opt/ros/kinetic
TMP_ROOT=/tmp/utcoupe_ws
TMP_SRC=$TMP_ROOT/src

# Fill this structure to add other external nodes
declare -A external_nodes
external_nodes=(
	["teraranger"]="https://github.com/Terabee/teraranger"
	["processing_lidar_objects"]="https://github.com/utcoupe/obstacle_detector.git"
)

function fetch_missing_nodes() {
	mkdir -p $TMP_SRC
	cd $TMP_SRC
	for node_name in "${!external_nodes[@]}"; do 
		if [ ! -d $INSTALL_ROOT/share/$node_name ]; then
			echo "$node_name is not installed, fetch it."
			git clone "${external_nodes[$node_name]}"
		else
			echo "$node_name is already installed installed."
		fi
	done
}

function compile_and_install_nodes() {
	if [ ! -z "$(ls -A $TMP_SRC)" ]; then
		cd $TMP_ROOT
		# Kind of ideal command to use, but catkin wont install direclty in ros base workspace because of _setup_util.py file...
		#sudo -u $USER -H bash -c "source $INSTALL_ROOT/setup.bash; catkin_make install -DCMAKE_INSTALL_PREFIX=$INSTALL_ROOT -DCMAKE_BUILD_TYPE=Release"
		#TODO improve using jobs directly (get the number of the current computer)
		catkin_make install -DCMAKE_BUILD_TYPE=Release
		sudo cp -ar $TMP_ROOT/install/include/* $INSTALL_ROOT/include
		sudo cp -ar $TMP_ROOT/install/share/* $INSTALL_ROOT/share
		sudo cp -ar $TMP_ROOT/install/lib/* $INSTALL_ROOT/lib
	fi
}

pushd . > /dev/null
fetch_missing_nodes
compile_and_install_nodes
popd > /dev/null

