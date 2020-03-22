cmake_minimum_required(VERSION 2.8.3)
project(navigator)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    rospy
    std_msgs
    geometry_msgs
    visualization_msgs
    actionlib
    actionlib_msgs
    pathfinder
    ard_asserv
    static_map
    tf
    interactive_markers
    game_manager
    utcoupe_metrics
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
    DIRECTORY src/asserv_client src/collisions_client src/localizer_client src/map_client src/pathfinder_client
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    PATTERN "*.pyc" EXCLUDE
)

#############
## Testing ##
#############

