#!/bin/bash

PROJECT_NAME="${PWD##*/}" # assuming directory name == cmake project name

function red_echo() {
	echo -e "\033[31m$1\033[0m"
}

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

function generate_control_lib_for_simu() {
    if [ ! "${PWD##*/}" = "stm32_asserv" ]; then
		red_echo "Please run this script from the stm32_asserv folder. Asserv library cannot be generated."
        return
    fi
    gcc -c -Wall -Werror -fPIC -I"${PWD}/include" "${PWD}/src/_shared_control.c"
    gcc -c -Wall -Werror -fPIC -I"${PWD}/include" "${PWD}/src/_shared_local_math.c"
    gcc -c -Wall -Werror -fPIC -I"${PWD}/include" "${PWD}/src/_shared_goals.c"
    gcc -c -Wall -Werror -fPIC -I"${PWD}/include" "${PWD}/src/_shared_PID.c"
    gcc -c -Wall -Werror -fPIC -I"${PWD}/include" "${PWD}/src/_shared_protocol.c"
    gcc -c -Wall -Werror -fPIC -I"${PWD}/include" "${PWD}/src/_shared_robotstate.c"

    gcc -shared -o "${UTCOUPE_WORKSPACE}/libs/lib_stm32_asserv.so" \
     "${PWD}/_shared_control.o" "${PWD}/_shared_local_math.o" "${PWD}/_shared_goals.o" \
     "${PWD}/_shared_PID.o" "${PWD}/_shared_protocol.o" "${PWD}/_shared_robotstate.o"

    rm "${PWD}/_shared_control.o" "${PWD}/_shared_local_math.o" "${PWD}/_shared_goals.o" \
     "${PWD}/_shared_PID.o" "${PWD}/_shared_protocol.o" "${PWD}/_shared_robotstate.o"
}

function upload_program() {
    st-flash --format ihex write "${PROJECT_NAME}.hex"
}

generate_cmake
compile_program
upload_program
cd ..
generate_control_lib_for_simu
