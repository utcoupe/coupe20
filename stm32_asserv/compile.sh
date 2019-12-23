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

function generate_control_lib_for_simu() {
    gcc -c -Wall -Werror -fPIC -I"${PWD}/include" "${PWD}/src/control_shared.c"
    gcc -c -Wall -Werror -fPIC -I"${PWD}/include" "${PWD}/src/local_math.c"
    gcc -c -Wall -Werror -fPIC -I"${PWD}/include" "${PWD}/src/goals.c"
    gcc -c -Wall -Werror -fPIC -I"${PWD}/include" "${PWD}/src/PID.c"
    gcc -c -Wall -Werror -fPIC -I"${PWD}/include" "${PWD}/simu_only_functions/simu_functions.c"

    gcc -shared -o "${UTCOUPE_WORKSPACE}/libs/lib_asserv_control_shared.so" \
     "${PWD}/control_shared.o" "${PWD}/local_math.o" "${PWD}/goals.o" \
     "${PWD}/PID.o" "${PWD}/getters.o"
    rm "${PWD}/control_shared.o" "${PWD}/local_math.o" "${PWD}/goals.o" \
     "${PWD}/PID.o" "${PWD}/getters.o"
}

function upload_program() {
    st-flash --format ihex write "${PROJECT_NAME}.hex"
}

generate_cmake
compile_program
upload_program
cd ..
generate_control_lib_for_simu
