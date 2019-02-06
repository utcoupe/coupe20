cmake_minimum_required(VERSION 2.8.3)
project(geometry_tools)

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
    roscpp
    roslib
    geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES geometry_tools
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

file (
    GLOB_RECURSE
    source_files_geometry_tools
    src/geometry_tools/*
)

add_library(
    ${PROJECT_NAME}
    ${source_files_geometry_tools}
)

add_dependencies(
    ${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


#############
## Testing ##
#############

