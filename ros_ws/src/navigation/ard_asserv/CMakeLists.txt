cmake_minimum_required(VERSION 2.8.3)
project(ard_asserv)
enable_language(C CXX)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
    message_generation
    genmsg
    actionlib_msgs
    actionlib
    port_finder
    game_manager
    static_map
    tf
    tf2_ros
)

add_compile_options(
    -std=c11

    -Wall
    -Wextra # reasonable and standard
    -Wshadow # warn the user if a variable declaration shadows one from a
             # parent context
    -Wcast-align # warn for potential performance problem casts
    -Wunused # warn on anything being unused
    -Wpedantic # warn if non-standard C++ is used
    -Wconversion # warn on type conversions that may lose data
    -Wsign-conversion # warn on sign conversions
    -Wnull-dereference # warn if a null dereference is detected
    -Wdouble-promotion # warn if float is implicit promoted to double
    -Wformat=2  # warn on security issues around functions that format output
                # (ie printf)
    # Uniquement pour GCC :
    -Wmisleading-indentation   # warn if identation implies blocks where blocks
                               # do not exist
    -Wduplicated-cond # warn if if / else chain has duplicated conditions
    -Wduplicated-branches # warn if if / else branches have duplicated code
    -Wlogical-op # warn about logical operations being used where bitwise were
                 # probably wanted
)

## Generate messages
add_message_files(
    FILES
    RobotSpeed.msg
)

## Generate services
add_service_files(
    FILES
    Parameters.srv
    EmergencyStop.srv
    Goto.srv
    Management.srv
    Pwm.srv
    SetPos.srv
    Speed.srv
)

## Generate actions
add_action_files(DIRECTORY action FILES DoGoto.action)

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

## Include and compile asserv shared files
add_subdirectory("$ENV{UTCOUPE_WORKSPACE}/asserv/shared/" utcoupe_asserv_shared)

include_directories(
#    ${catkin_INCLUDE_DIRS}
    ${UTCOUPE_ASSERV_SHARED_INCLUDE_DIR}
)

add_library(utcoupe_shared_asserv SHARED
    ${UTCOUPE_ASSERV_SHARED_SOURCES}
)

#############
## Install ##
#############

catkin_install_python(
    PROGRAMS src/ard_asserv_node.py src/marker_server_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install (
    DIRECTORY src/asserv
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    PATTERN "*.pyc" EXCLUDE
)