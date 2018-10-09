cmake_minimum_required(VERSION 2.8.3)
project(movement_actuators)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    actionlib_msgs
    tf2_ros
    geometry_msgs
    actionlib
    ai_game_manager
)

# Generate services in the 'srv' folder
add_service_files(
    FILES
    ActivateCanon.srv
)

## Generate actions in the 'action' folder
add_action_files(DIRECTORY action FILES
    Dispatch.action
    Arm.action
    Barrel.action
    CubePicker.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS std_msgs actionlib_msgs tf2_ros rospy actionlib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(
    PROGRAMS src/actuators_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY src/actuators
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    PATTERN "*.pyc" EXCLUDE
)
