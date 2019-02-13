cmake_minimum_required(VERSION 2.8.3)
project(collisions)

add_compile_options(
    -std=c++14
    
#    -Wall
#    -Wextra
#    -Wpedantic
#    -Wshadow
#    -Wnon-virtual-dtor
    
#     -Wold-style-cast
#     -Wcast-align
#     -Wunused
#     -Woverloaded-virtual
#     -Wconversion
#     -Wsign-conversion
#     -Wmisleading-indentation
#     -Wduplicated-cond
#     -Wduplicated-branches
#     -Wlogical-op
#     -Wnull-dereference
#     -Wuseless-cast
#     -Wdouble-promotion
    
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    roslib
    std_msgs
    geometry_msgs
    genmsg
    game_manager
    navigator
    ard_asserv
    objects_classifier
    tf2
    tf2_ros
    tf
    visualization_msgs
    static_map
)

##############
## Messages ##
##############

# Generate messages
add_message_files(
    FILES
    PredictedCollision.msg
)

# Generate services
add_service_files(
    FILES
    ActivateCollisions.srv
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
    CATKIN_DEPENDS rospy std_msgs message_runtime roslib
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

file(
    GLOB_RECURSE
    source_files_collisions
    src/collisions/*
)

add_executable(collisions_node ${source_files_collisions} src/collisions_node.cpp)

target_link_libraries(collisions_node
    ${catkin_LIBRARIES}
)

add_dependencies(collisions_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

install(
    TARGETS collisions_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY include/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############
