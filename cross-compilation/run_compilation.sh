#!/bin/bash

# TODO select architecture
compile_arch="armv7"

img_tag="utcoupe-ros-kinetic-${compile_arch}"
img_dir="utcoupe-ros-kinetic-${compile_arch}"

img_ws_root_dir="/utcoupe/coupe18"

cross_compilation_dir="${UTCOUPE_WORKSPACE}/cross-compilation"
cross_compilation_install_dir="${cross_compilation_dir}/generated_install/${compile_arch}"

docker build -t ${img_tag} "${cross_compilation_dir}/${img_dir}"

mkdir -p "${cross_compilation_install_dir}"

# looks slower than the other method
# docker run \
#     -i \
#     --mount type=bind,source="${UTCOUPE_WOKSPACE}",target="${img_ws_root_dir}" \
#     ${img_tag}

echo "Generating files in \"${cross_compilation_install_dir}\""

docker run \
    -i \
    --mount type=bind,source="${UTCOUPE_WORKSPACE}"/ros_ws/src,target="${img_ws_root_dir}"/ros_ws/src,readonly \
    --mount type=bind,source="${UTCOUPE_WORKSPACE}"/libs,target="${img_ws_root_dir}"/libs,readonly \
    --mount type=bind,source="${cross_compilation_install_dir}",target="${img_ws_root_dir}"/ros_ws/install \
    ${img_tag}
# Makes cmake crash
#    --tmpfs "${img_ws_root_dir}"/ros_ws/devel
#    --tmpfs "${img_ws_root_dir}"/ros_ws/build

echo "Creating archive..."
last_directory=$(pwd)
cd "${cross_compilation_install_dir}/../"
tar -czf  "${compile_arch}.tgz" "${compile_arch}"
cd "${last_directory}"

echo "DONE, ENJOY THE CROSS-COMPILED BINARIES!"
