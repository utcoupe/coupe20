cmake_minimum_required(VERSION 2.8)

#todo include it in utcoupe workspace !
set(CMAKE_TOOLCHAIN_FILE $ENV{UTCOUPE_WORKSPACE}/libs/arduino-cmake/cmake/ArduinoToolchain.cmake) # Arduino Toolchain

set(ARDUINO_CXX_FLAGS    "${ARDUINO_C_FLAGS} -std=c++11")

# target are : arduino_r_t with r = pr/gr and t = asserv/others
project(arduino C CXX)

set(ARDUINO_DEFAULT_BOARD mega2560)

SET(TARGET_ARDUINO "mega2560" CACHE STRING "User defined arduino target")
SET(TARGET_ROBOT "gr" CACHE STRING "User defined robot")
SET(TARGET_PROGRAM "asserv" CACHE STRING "User defined program")

#todo use the script to determine which port to use
if (TARGET_ARDUINO STREQUAL "nano328")
	set(ARDUINO_DEFAULT_PORT /dev/ttyUSB0)
else()
	set(ARDUINO_DEFAULT_PORT /dev/ttyACM0)
endif()

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../bin)

#set(TARGET_ROBOT gr)
#set(TARGET_PROGRAM asserv)

file(GLOB_RECURSE SRC_FILES	${TARGET_ROBOT}/${TARGET_PROGRAM}/*.cpp ${TARGET_ROBOT}/${TARGET_PROGRAM}/*.c)
include_directories(${TARGET_ROBOT}/${TARGET_PROGRAM} common/${TARGET_PROGRAM} common/shared)

#asserv or others extra common stuff
#set(EXTRA_SRC_FILES "")
file(GLOB_RECURSE EXTRA_SRC_FILES common/${TARGET_PROGRAM}/*.c common/${TARGET_PROGRAM}/*.cpp common/shared/*.cpp)
#if (TARGET_PROGRAM STREQUAL "asserv")
#	message("asserv program detected")
#	file(GLOB_RECURSE EXTRA_SRC_FILES common/${TARGET_PROGRAM}/*.c common/${TARGET_PROGRAM}/*.cpp)
#endif()

generate_arduino_firmware(${TARGET_ROBOT}_${TARGET_PROGRAM}_${TARGET_ARDUINO}
    SRCS ${SRC_FILES} ${EXTRA_SRC_FILES}
#    HDRS ${HDR_FILES}
#	LIBS HardwareSerial
    PORT ${ARDUINO_DEFAULT_PORT}
    BOARD ${TARGET_ARDUINO}
)

