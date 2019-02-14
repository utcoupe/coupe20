cmake_minimum_required(VERSION 2.8.3)
project(scheduler)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation

    ard_hmi
    game_manager
    static_map
    navigator
    ard_tower
    ard_asserv
    ax12
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
    FILES
    TaskResult.msg
    AIScore.msg
)

add_service_files(
    FILES
    test.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
	CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(
    PROGRAMS src/scheduler_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install (
    PROGRAMS src/scheduler_communication.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY src/ai
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    PATTERN "*.pyc" EXCLUDE
)

#############
## Testing ##
#############
