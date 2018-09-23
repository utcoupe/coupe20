#!/bin/bash

USER_TARGET=""
USER_ROBOT=""
USER_PROGRAM=""

function generate_cmake() {
	rm -rf build
	mkdir -p build
	cd build
	cmake ../ -DTARGET_ARDUINO:STRING=$USER_TARGET -DTARGET_ROBOT:STRING=$USER_ROBOT -DTARGET_PROGRAM:STRING=$USER_PROGRAM
}

function compile_program() {
	if [ ! "${PWD##*/}" = "build" ]; then
		printf "Not in the right place for build..."
	fi
	make
}

function upload_program() {
	#TODO check if an arduino is present
	make upload	
}

function ask_user() {
	printf "What's the target (nano328, uno, leonardo, mega2560) ?"
	read answer
	#TODO shorter names ? (eg mega...)
	if [ "$answer" = "nano328" ] || [ "$answer" = "uno" ] || [ "$answer" = "leonardo" ] || [ "$answer" = "mega2560" ]; then
		USER_TARGET="$answer"
	fi
	
	printf "What's the robot (pr, gr) ?"
	read answer
	if [ "$answer" = "pr" ] || [ "$answer" = "gr" ]; then
		USER_ROBOT="$answer"
	fi
	
	printf "What's the program (asserv, others) ?"
	read answer
	if [ "$answer" = "asserv" ] || [ "$answer" = "others" ]; then
		USER_PROGRAM="$answer"
	fi
	
	if [ "$USER_ROBOT" = "" ] || [ "$USER_ROBOT" = "" ] || [ "$USER_PROGRAM" = "" ]; then
		printf "Incorrect parameters, please relaunch the script with correct parameters...\n"
		exit 1
	fi
}

# "Main"
ask_user
generate_cmake
compile_program
upload_program
