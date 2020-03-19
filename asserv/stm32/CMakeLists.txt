set(TOOLCHAIN_PREFIX "$ENV{UTCOUPE_WORKSPACE}/libs/gcc-arm-none-eabi-6-2017-q2-update/")

set(STM32_CHIP STM32F303K8)
# TODO copy or symlink in $UTCOUPE_WORKSPACE OR regex
set(STM32Cube_DIR "$ENV{HOME}/STM32Cube/Repository/STM32Cube_FW_F3_V1.11.0/")
SET(CMAKE_TOOLCHAIN_FILE "$ENV{UTCOUPE_WORKSPACE}/libs/stm32-cmake/cmake/gcc_stm32.cmake")

SET(STM32_LINKER_SCRIPT "STM32F303K8Tx_FLASH.ld")

project(stm32_asserv)
cmake_minimum_required(VERSION 2.8)

enable_language(C CXX ASM)

FIND_PACKAGE(CMSIS REQUIRED)
# STM32 COMPONENTS = header names
FIND_PACKAGE(STM32HAL COMPONENTS gpio tim uart dma rcc pwr cortex flash REQUIRED)

include($ENV{UTCOUPE_WORKSPACE}/asserv/shared/shared_asserv.cmake)

add_compile_options(
        -fno-exceptions # prevents huge overflow TODO find source
        -Wall
        -Wextra
        -Wshadow
#        -Wnon-virtual-dtor
        -pedantic
#        -Wcast-align
#        -Wuseless-cast
        -Wdouble-promotion
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMSIS_INCLUDE_DIRS}
    ${STM32HAL_INCLUDE_DIR}
    ${UTCOUPE_ASSERV_SHARED_INCLUDE_DIR}
    include
    asm
)

file(
    GLOB_RECURSE
    source_files
    src/*
)

#set_source_files_properties(startup_stm32f303x8.s PROPERTIES COMPILE_FLAGS ${CMAKE_ASM_FLAGS})

add_executable(${CMAKE_PROJECT_NAME}
    src/main.cpp
    ${source_files}
    ${UTCOUPE_ASSERV_SHARED_SOURCES}
    ${STM32HAL_SOURCES}
    ${CMSIS_SOURCES}
#    startup_stm32f303x8.s
)

STM32_SET_TARGET_PROPERTIES(${CMAKE_PROJECT_NAME})
STM32_ADD_HEX_BIN_TARGETS(${CMAKE_PROJECT_NAME})

