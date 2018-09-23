#!/bin/bash

### The goal of this script is to install all UTCoupe specific packages to have a working setup.
### When you just cloned the UTCoupe repository, you have to run this script first !

function green_echo() {
	echo -e "\033[32m$1\033[0m"
}

function red_echo() {
	echo -e "\033[31m$1\033[0m"
}

# Globals
ARCH=$(uname -m)

### Install the linux packages
function install_apt() {
	green_echo "Install missing packages..."
	sudo apt-get install git build-essential python python-pip cmake libboost-dev libsdl1.2-dev gcc-avr avrdude avr-libc libsfml-dev libarmadillo-dev libavcodec-dev libswscale-dev

	# Check if it's a PC or a raspi
	if [ "$ARCH" = "x86_64" ]; then
		green_echo "x86 architecture detected."
		sudo apt-get install nodejs npm nodejs-legacy linux-headers-$(uname -r)
	elif [ "$ARCH" = "armv7l" ]; then
		green_echo "Raspberry Pi 3 system detected, remove previous npm installation to setup the used version."
		sudo apt-get install raspberrypi-kernel-headers
		sudo apt-get remove npm nodejs nodejs-legacy
		curl -sL https://deb.nodesource.com/setup_4.x | sudo -E bash -
		sudo npm install npm@3.5.2 -g
		#TODO be sure that all is correct for raspi
	elif [ "$ARCH" = "armv6l" ]; then
		sudo apt-get install raspberrypi-kernel-headers
		sudo apt-get remove npm nodejs nodejs-legacy
		wget https://nodejs.org/dist/v4.8.1/node-v4.8.1-linux-armv6l.tar.gz
		tar -xvf node-v4.8.1-linux-armv6l.tar.gz
		cd node-v4.8.1-linux-armv6l
		sudo cp -R * /usr/local/
		cd ..
		sudo rm -rf node-v4.8.1-linux-armv6l
	else
		red_echo "ARCH of system not corresponding to a know arch... ($ARCH)"
	fi
}

### Install ros-desktop-full
function install_ros() {
	if [ "$(lsb_release -sc)" = "xenial" ] || [ "$(lsb_release -sc)" = "willy" ]; then
		sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
		sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
		sudo apt-get update
		# "Install" Arduino libs needed by us
		if [ -d "/usr/share/arduino" ] && [ -d "$PWD/libs/arduino-libraries" ]; then
			sudo cp -ar $PWD/libs/arduino-libraries/* /usr/share/arduino/libraries/
		else
			red_echo "Unable to locate arduino folder or arduino-libraries extracted folder, please check your setup"
		fi
		sudo rosdep init
		rosdep update
	else
		red_echo "Your OS is not Ubuntu Willy or Xenial, ROS will not been installed..."
	fi
}

function install_ros_depencies() {
	if [ "$ARCH" = "x86_64" ]; then
		sudo apt-get install ros-kinetic-desktop-full ros-kinetic-rosserial-arduino ros-kinetic-rosbridge-suite ros-kinetic-tf2-web-republisher ros-kinetic-serial ros-kinetic-dynamixel-sdk ros-kinetic-rosserial-python ros-kinetic-urg-c ros-kinetic-urg-node
	elif [ "$ARCH" = "armv7l" ]; then
		sudo apt-get install ros-kinetic-ros-base ros-kinetic-tf2 ros-kinetic-tf2-ros ros-kinetic-rviz ros-kinetic-diagnostic-updater ros-kinetic-roslint ros-kinetic-camera-info-manager ros-kinetic-rosserial-arduino ros-kinetic-rosbridge-suite ros-kinetic-tf2-web-republisher ros-kinetic-serial ros-kinetic-dynamixel-sdk ros-kinetic-rosserial-python ros-kinetic-urg-c ros-kinetic-urg-node
	fi
}

### Setup the variable environment to taget the UTCoupe main folder
### This function use $PWD instead of $UTCOUPE_WORKSPACE because if the env variable is not set before running the script, $UTCOUPE_WORKSPACE is unknown
function env_setup() {
	# Add the UTCOUPE_WORKSPACE env variable, default consider as bash shell
	if [ -z "$UTCOUPE_WORKSPACE" ]; then
		green_echo "Env variable is not set."
		if [ "$SHELL" = "/bin/zsh" ]; then
			echo "export UTCOUPE_WORKSPACE=$PWD" >> $HOME/.zshrc
			echo "export ROS_LANG_DISABLE=genlisp:geneus" >> $HOME/.zshrc
			echo "export ROSCONSOLE_FORMAT='[\${node}:\${severity}] [\${time}] : \${message}'" >> $HOME/.zshrc
			red_echo "Warning :\n"
			red_echo "Please \"source ~/.zshrc\" and run again this script if necessary\n"
			exit 1
		else
			echo "export UTCOUPE_WORKSPACE=$PWD" >> $HOME/.bashrc
			echo "export ROS_LANG_DISABLE=genlisp:geneus" >> $HOME/.bashrc
			echo "export ROSCONSOLE_FORMAT='[\${node}:\${severity}] [\${time}] : \${message}'" >> $HOME/.bashrc
            		source $HOME/.bashrc
		fi
	fi
	# Add the current user to the dialout group (to r/w in /dev files)
	if ! id -Gn $USER | grep -qw "dialout"; then
	        sudo usermod -a -G dialout $USER
	fi
	# Untar all libraries (the copy is done in install_ros function)
	for f in $PWD/libs/*; do
		if [ ! -d $f ]; then
			tar -C $PWD/libs -xzf $f
		fi
	done
	# Add the Ethernet IP address of raspberry pi to have a shortcut
	if ! grep "utcoupe" /etc/hosts > /dev/null; then
	    sudo sh -c "echo '#UTCoupe raspberry pi Ethernet IP when connected on the UTC network\n172.18.159.254	utcoupe_rpi31\n172.18.161.161	utcoupe_rpi32\n172.18.161.162	utcoupe_rpi33' >> /etc/hosts"
	fi
	# Update the submodules
	git submodule update --init --recursive
}

### Then install the UTCoupe ROS workspace
function install_ros_workspace() {
	# Install the UTCoupe ROS specific packages
	#TODO use the requirements system
	pip install pyserial numpy scipy pymongo pyclipper pillow
	printf "Download, compile and install all ROS external nodes needed by UTCoupe, this may take a while."
	./scripts/install_external_nodes.sh
}

### Main install_script function, ask the user to install each main components
function launch_script() {

	env_setup

	printf "Install apt missing packets ? [Y/n]?"
	read answer
	if [ "$answer" = "" ] || [ "$answer" = "y" ] || [ "$answer" = "Y" ]; then
		install_apt
		if [ ! -d "/opt/ros" ]; then
			printf "ROS has not been detected in /opt/ros, launch the installation process..."
			install_ros
		fi
		install_ros_depencies
	fi
	printf "Install UTCoupe ROS workspace (ROS must to be installed) ? [Y/n]?"
	read answer
	if [ "$answer" = "" ] || [ "$answer" = "y" ] || [ "$answer" = "Y" ]; then
		install_ros_workspace
	fi
}

# Check that the folder has been cloned from git and not downloaded, because submodules won't work...
if [ ! -d ".git" ]; then
	red_echo "You have to clone this repository from git, not download it."
	exit 1
fi

# Verify that the script is launched from the right place
if [ ! "${PWD##*/}" = "coupe18" ]; then
	red_echo "You have to launch this script from UTCoupe main directory : ./script/${0##*/} or to rename this folder in coupe18."
	exit 1
fi

# Ask the user if he wants launch the script
printf "Launch install script ? [Y/n]?"
read answer
if [ "$answer" = "" ] || [ "$answer" = "y" ] || [ "$answer" = "Y" ]; then
	launch_script
	echo "If you run the install script for the first time, please reboot your computer to apply all modifications."
fi
