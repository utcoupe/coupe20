cmake_minimum_required(VERSION 2.8.3)
project(navigation_navigator)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    rospy
    std_msgs
    geometry_msgs
    visualization_msgs
    actionlib
    actionlib_msgs
    navigation_pathfinder
    drivers_ard_asserv
    memory_map
    tf
    interactive_markers
    ai_game_manager
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
    FILES
    Status.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    Goto.srv
)

## Generate actions in the 'action' folder
add_action_files(
    FILES
    DoGoto.action
    DoGotoWaypoint.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
#    CATKIN_DEPENDS rospy std_msgs message_runtime
)

###########
## Build ##
###########

#############
## Install ##
#############

catkin_install_python(
    PROGRAMS src/navigator_node.py src/marker_server_node.py src/planning.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY src/asserv src/collisions src/localizer src/map src/pathfinder
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    PATTERN "*.pyc" EXCLUDE
)

#############
## Testing ##
#############

