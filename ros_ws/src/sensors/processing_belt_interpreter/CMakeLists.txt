cmake_minimum_required(VERSION 2.8.3)
project(processing_belt_interpreter)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    geometry_msgs
    std_msgs
    tf2
    tf2_ros
    rospy
    dynamic_reconfigure
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
    FILES
    BeltRects.msg
    RectangleStamped.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
)

generate_dynamic_reconfigure_options(
    cfg/BeltInterpreter.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS message_runtime
)

#############
## Install ##
#############

catkin_install_python(
    PROGRAMS src/belt_interpreter_node.py src/belt_parser.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY cfg
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "*.pyc" EXCLUDE
)
