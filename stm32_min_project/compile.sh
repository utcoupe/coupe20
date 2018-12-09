#!/bin/bash

PROJECT_NAME="${PWD##*/}" # assuming directory name == cmake project name

function generate_cmake() {
    rm -rf build
    mkdir -p build
    cd build
    cmake ..
}

function compile_program() {
	if [ ! "${PWD##*/}" = "build" ]; then
		printf "Not in the right place for build..."
	fi
	make
}

function upload_program() {
    echo "Upload not implemented yet..."
}

generate_cmake
compile_program
upload_program
cd ..
