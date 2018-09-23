cmake_minimum_required(VERSION 2.8.3)
project(recognition_localizer)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    rospy
    tf2
    tf2_ros)

catkin_package(

)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(
    PROGRAMS src/localizer_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
