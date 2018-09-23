cmake_minimum_required(VERSION 2.8.3)
project(navigation_pathfinder)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    genmsg
    geometry_msgs
    dynamic_reconfigure
    ai_game_manager
    memory_map
    recognition_objects_classifier
)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH}) # Please note the use of CMAKE_CURRENT_SOURCE_DIR
find_package(SFML 2 REQUIRED COMPONENTS
    graphics
)

set(NLOHMANN_JSON_HEADER_LOCATION "$ENV{UTCOUPE_WORKSPACE}/libs/json/single_include/")

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
    FILES
    FindPath.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
    def/PathfinderNode.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS message_runtime roslib
)

###########
## Build ##
###########

include_directories(
    include
    ${NLOHMANN_JSON_HEADER_LOCATION}
    ${catkin_INCLUDE_DIRS}
)


file(
    GLOB_RECURSE
    source_files_pathfinder
    src/pathfinder/*
)
add_executable(pathfinder_node ${source_files_pathfinder} src/pathfinder_node.cpp)

target_link_libraries(pathfinder_node
    ${catkin_LIBRARIES}
    ${SFML_LIBRARIES} ${SFML_DEPENDENCIES}
)

add_dependencies(pathfinder_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

install (
    TARGETS pathfinder_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY include/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(
    DIRECTORY def/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/def
)

#############
## Testing ##
#############

