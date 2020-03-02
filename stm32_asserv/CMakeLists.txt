set(TOOLCHAIN_PREFIX "$ENV{UTCOUPE_WORKSPACE}/libs/gcc-arm-none-eabi-6-2017-q2-update/")

set(STM32_CHIP STM32F303K8)
set(STM32_LINKER_SCRIPT "STM32F303K8Tx_FLASH.ld")

# TODO copy or symlink in $UTCOUPE_WORKSPACE OR regex
set(STM32Cube_DIR "$ENV{HOME}/STM32Cube/Repository/STM32Cube_FW_F3_V1.11.0/")
SET(CMAKE_TOOLCHAIN_FILE "$ENV{UTCOUPE_WORKSPACE}/libs/stm32-cmake/cmake/gcc_stm32.cmake")

project(stm32_asserv)
cmake_minimum_required(VERSION 2.8)

enable_language(C CXX ASM)

FIND_PACKAGE(CMSIS REQUIRED)
# STM32 COMPONENTS = header names
FIND_PACKAGE(STM32HAL COMPONENTS gpio tim uart dma rcc pwr cortex flash REQUIRED)

add_compile_options(
        -Wall
        -Wextra
        -Wshadow
#        -Wnon-virtual-dtor
        -pedantic
#        -Wcast-align
#        -Wuseless-cast
        -Wdouble-promotion
#        -Os
)

add_link_options(
    --specs=nano.specs # fixes huge overflow
    --specs=nosys.specs # fixes newlib missing system calls
)

# Color output in Ninja
option (FORCE_COLORED_OUTPUT "Always produce ANSI-colored output (GNU/Clang only)." TRUE)
if (${FORCE_COLORED_OUTPUT})
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
       add_compile_options (-fdiagnostics-color=always)
    elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
       add_compile_options (-fcolor-diagnostics)
    endif ()
endif ()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMSIS_INCLUDE_DIRS}
    ${STM32HAL_INCLUDE_DIR}
    include
)

file(
    GLOB_RECURSE
    source_files
    src/*
)

add_executable(${CMAKE_PROJECT_NAME}
    src/main.cpp
    ${source_files}
    ${STM32HAL_SOURCES}
    ${CMSIS_SOURCES}
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    nosys
)

STM32_SET_TARGET_PROPERTIES(${CMAKE_PROJECT_NAME})
STM32_ADD_HEX_BIN_TARGETS(${CMAKE_PROJECT_NAME})
STM32_PRINT_SIZE_OF_TARGETS(${CMAKE_PROJECT_NAME})

