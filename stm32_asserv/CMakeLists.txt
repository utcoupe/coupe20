# To run this cmake, you need the packages :
# - gcc-arm-none-eabi (tested with ver 6.3.1)
# - libnewlib-arm-none-eabi (warning: check version compatibility, version in apt is obsolete in Ubuntu 18.04.1)
# - libstdc++-arm-none-eabi-newlib (warning: check version compatibility, version in apt is obsolete in Ubuntu 18.04.1)

set(STM32_CHIP STM32F303K8)
# TODO copy or symlink in $UTCOUPE_WORKSPACE OR regex
set(STM32Cube_DIR "$ENV{HOME}/STM32Cube/Repository/STM32Cube_FW_F3_V1.10.0/")
SET(CMAKE_TOOLCHAIN_FILE "$ENV{UTCOUPE_WORKSPACE}/libs/stm32-cmake/cmake/gcc_stm32.cmake")

project(stm32_asserv)
cmake_minimum_required(VERSION 2.8)

enable_language(C CXX ASM)

FIND_PACKAGE(CMSIS REQUIRED)
# STM32 COMPONENTS = header names
FIND_PACKAGE(STM32HAL COMPONENTS gpio tim uart dma rcc flash pwr cortex i2c can REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMSIS_INCLUDE_DIRS}
    ${STM32HAL_INCLUDE_DIR}
    include
    asm
)

file(
    GLOB_RECURSE
    source_files
    src/*
    include/*
    asm/*
)

set_source_files_properties(startup_stm32f303x8.s PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")

add_executable(${CMAKE_PROJECT_NAME}
    ${source_files}
    ${STM32HAL_SOURCES}
    startup_stm32f303x8.s
)

STM32_SET_TARGET_PROPERTIES(${CMAKE_PROJECT_NAME})
STM32_ADD_HEX_BIN_TARGETS(${CMAKE_PROJECT_NAME})

