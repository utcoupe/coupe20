#!/bin/bash

TMP_INSTALL="/tmp/utcoupe_install"

function green_echo() {
	echo -e "\033[32m$1\033[0m"
}

function update_dependancies () {
    green_echo "Manualy updating dependancies since apt version is wrong"
    mkdir -p $TMP_INSTALL
    cd $TMP_INSTALL
    wget "http://fr.archive.ubuntu.com/ubuntu/pool/universe/n/newlib/libnewlib-arm-none-eabi_3.0.0.20180802-2_all.deb"
    wget "http://fr.archive.ubuntu.com/ubuntu/pool/universe/n/newlib/libnewlib-dev_3.0.0.20180802-2_all.deb"
    wget "http://fr.archive.ubuntu.com/ubuntu/pool/universe/libs/libstdc++-arm-none-eabi/libstdc++-arm-none-eabi-newlib_7-2018-q2-3+11_all.deb"
    sudo dpkg -i *.deb
}

function install_default_dep () {
    green_echo "Installing from apt..."
    sudo apt update -qq && sudo apt install -qq -y gcc-arm-none-eabi openocd cmake
    
    # TODO test compatibility on Ubuntu 16.04
    if [ "$(lsb_release -sc)" = "bionic" ]; then
        # libnewlib-arm-none-eabi and libstdc++-arm-none-eabi-newlib in apt packages are buggy
        # https://bugs.launchpad.net/ubuntu/+source/gcc-arm-none-eabi/+bug/1767223
        update_dependancies
    fi
}

function install_stlink_utils () {
    green_echo "Installing stlink v2.1..."
    rm -rf $TMP_INSTALL
    mkdir -p $TMP_INSTALL
    cd $TMP_INSTALL
    git clone https://github.com/texane/stlink
    cd stlink
    make realease && cd build/Release && sudo make install
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
