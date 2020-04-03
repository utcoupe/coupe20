cmake_minimum_required(VERSION 3.0.2) # kinetic requirement
project(utcoupe_metrics)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
)

catkin_python_setup()

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
    LIBRARIES utcoupe_metrics
    CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}_warnings INTERFACE)
add_library(${PROJECT_NAME}_options INTERFACE)

# For modern cmake
# target_compile_features(${PROJECT_NAME}_options INTERFACE cxx_std_14)
# But is not available for this version (3.0)
target_compile_options(${PROJECT_NAME}_options INTERFACE
    -std=c++14
)

target_include_directories(${PROJECT_NAME}_options INTERFACE
    include
    ${catkin_INCLUDE_DIRS}
)

option(ENABLE_ASAN "Enable address sanitizer" FALSE)
if(ENABLE_ASAN)
  target_compile_options(${PROJECT_NAME}_options INTERFACE -fsanitize=address)
  target_link_libraries(${PROJECT_NAME}_options INTERFACE -fsanitize=address)
endif()

target_compile_options(${PROJECT_NAME}_options INTERFACE
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wnon-virtual-dtor
    
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

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/${PROJECT_NAME}/metric_exporter.cpp
    src/${PROJECT_NAME}/metric_types/counter.cpp
    src/${PROJECT_NAME}/metric_types/gauge.cpp
    src/${PROJECT_NAME}/metric_types/chronometer.cpp
)

add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${PROJECT_NAME}_options
        ${PROJECT_NAME}_warnings
        ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## C++ Library

install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
    DIRECTORY include/${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

## Python Library

# Nothing todo here, see setup.py

#############
## Testing ##
#############


