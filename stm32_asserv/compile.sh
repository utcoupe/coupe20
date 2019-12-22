#!/bin/bash

PROJECT_NAME="${PWD##*/}" # assuming directory name == cmake project name

function generate_cmake() {
    rm -rf build
    mkdir -p build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE="RELEASE"
}

function compile_program() {
	if [ ! "${PWD##*/}" = "build" ]; then
		printf "Not in the right place for build..."
	fi
	make && make "${PROJECT_NAME}.hex"
}

function generate_shared_library_for_simu() {
    cd src
    gcc -c -Wall -Werror -fpic "${PWD}/command_law.c"
    gcc -shared -o "${UTCOUPE_WORKSPACE}/libs/lib_asserv_command_law.so" "${PWD}/command_law.o"
    rm "${PWD}/command_law.o"
    cd ..
}

function upload_program() {
    st-flash --format ihex write "${PROJECT_NAME}.hex"
}

generate_shared_library_for_simu
generate_cmake
compile_program
upload_program
cd ..
