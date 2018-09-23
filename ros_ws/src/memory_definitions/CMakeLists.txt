cmake_minimum_required(VERSION 2.8.3)
project(memory_definitions)

find_package(catkin REQUIRED COMPONENTS message_generation)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
    FILES
    GetDefinition.srv
 )

generate_messages(
    DEPENDENCIES
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package()

###########
## Build ##
###########

include_directories()


#############
## Install ##
#############

catkin_install_python(
    PROGRAMS src/definitions_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY def/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/def
)

install(
    DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
