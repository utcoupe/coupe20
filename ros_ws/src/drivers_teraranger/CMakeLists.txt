cmake_minimum_required(VERSION 2.8.3)
project(drivers_teraranger)

find_package(catkin REQUIRED COMPONENTS
    drivers_port_finder
    drivers_ard_others
    ai_game_manager
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS
    drivers_port_finder
    drivers_ard_others
    sensor_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(
    PROGRAMS src/teraranger_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
