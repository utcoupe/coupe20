#!/bin/bash

TMP_INSTALL="/tmp/utcoupe_install"
GCC_ARM_NONE_EABI_ARCHIVE="gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2"

function green_echo() {
	echo -e "\033[32m$1\033[0m"
}

function install_default_dep () {
    green_echo "Decompressing ${GCC_ARM_NONE_EABI_ARCHIVE}"
    
    tar -C "${UTCOUPE_WORKSPACE}/libs" -xjf "${UTCOUPE_WORKSPACE}/libs/${GCC_ARM_NONE_EABI_ARCHIVE}"
}

function install_stlink_utils () {
    green_echo "Installing stlink v2.1..."
    rm -rf $TMP_INSTALL
    mkdir -p $TMP_INSTALL
    cd $TMP_INSTALL
    git clone https://github.com/texane/stlink
    cd stlink
    make release && cd build/Release && sudo make install
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
    # Specific to Ubuntu
    green_echo "Updating ld path..."
    echo "/usr/local/lib" > libstlink.conf
    sudo mv libstlink.conf /etc/ld.so.conf.d/
    sudo ldconfig
}

install_default_dep
# TODO install stm32f3xx lib
install_stlink_utils
