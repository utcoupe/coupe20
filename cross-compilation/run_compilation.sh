#!/bin/bash

# TODO select architecture
compile_arch="armv7"
# TODO ask user
build_image=O
ros_version='kinetic'

img_tag="utcoupe-ros-${ros_version}-${compile_arch}"
img_dir="utcoupe-ros-${ros_version}-${compile_arch}"

img_ws_root_dir="/utcoupe/coupe20"

cross_compilation_dir="${UTCOUPE_WORKSPACE}/cross-compilation"
cross_compilation_install_dir="${cross_compilation_dir}/generated_install/${ros_version}/${compile_arch}"

function green_echo() {
	echo -e "\033[32m$1\033[0m"
}

function red_echo() {
	echo -e "\033[31m$1\033[0m"
}

function build_image() {
    green_echo "Started to build $img_tag..."
    docker build -t ${img_tag} "${cross_compilation_dir}/${img_dir}"
    green_echo "Done."
}

function run_cross_compilation() {
    if [[ -z "$(docker images --format='{{print .Tag}}' | grep ${img_tag})" ]]; then
        red_echo "docker image ${img_tag} not found locally."
        if [[ ${build_image} -eq 0 ]]; then
            docker pull "utcoupe/coupe20:${img_tag}"
        else
            build_image
        fi
    fi

    mkdir -p "${cross_compilation_install_dir}"
    # looks slower than the other method
    # docker run \
    #     -i \
    #     --mount type=bind,source="${UTCOUPE_WOKSPACE}",target="${img_ws_root_dir}" \
    #     ${img_tag}

    green_echo "Generating files in \"${cross_compilation_install_dir}\"..."

    docker run \
        -i \
        --mount type=bind,source="${UTCOUPE_WORKSPACE}"/ros_ws/src,target="${img_ws_root_dir}"/ros_ws/src \
        --mount type=bind,source="${UTCOUPE_WORKSPACE}"/libs,target="${img_ws_root_dir}"/libs,readonly \
        --mount type=bind,source="${cross_compilation_install_dir}",target="${img_ws_root_dir}"/ros_ws/install \
        --mount type=bind,source="${UTCOUPE_WORKSPACE}/asserv",target$${img_ws_root_dir}/asserv,readonly \
        "utcoupe/coupe20:${img_tag}" \
        /bin/bash -c "catkin_make install -DCMAKE_BUILD_TYPE=Release"
    # Makes cmake crash
    #    --tmpfs "${img_ws_root_dir}"/ros_ws/devel
    #    --tmpfs "${img_ws_root_dir}"/ros_ws/build
    green_echo "Done."
}

function create_archive() {
    green_echo "Creating archive..."
    pushd .
    cd "${cross_compilation_install_dir}/../"
    tar -czf  "${compile_arch}.tgz" "${compile_arch}"
    popd
}

run_cross_compilation
create_archive

green_echo "DONE, ENJOY THE CROSS-COMPILED BINARIES!"
