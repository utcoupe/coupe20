cmake_minimum_required(VERSION 2.8.3)
project(memory_map)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
    message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
    FILES
    Waypoint.msg
)

# Generate services in the 'srv' folder
add_service_files(
    FILES
    MapGet.srv
    MapSet.srv
    MapTransfer.srv
    MapGetOccupancy.srv
    MapGetObjects.srv
    FillWaypoint.srv
)

# Generate added messages and services with any dependencies listed here
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
    PROGRAMS src/map_node.py src/map_communication.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY src/map_manager src/markers src/occupancy
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    PATTERN "*.pyc" EXCLUDE
)

install(
    DIRECTORY def
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
