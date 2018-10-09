cmake_minimum_required(VERSION 2.8.3)
project(drivers_port_finder)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    message_generation
    genmsg
)

## Generate services
add_service_files(
    FILES
    GetPort.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(
    PROGRAMS src/port_finder_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY def/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/def
)
