cmake_minimum_required(VERSION 2.8.3)
project(recognition_objects_classifier)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    geometry_msgs
    ai_game_manager
    processing_lidar_objects
    processing_belt_interpreter
    std_msgs
    tf2
    tf2_ros
    dynamic_reconfigure
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
    FILES
    CircleObstacleStamped.msg
    ClassifiedObjects.msg
    SegmentObstacleStamped.msg
)

generate_messages(
    DEPENDENCIES
    processing_lidar_objects
    processing_belt_interpreter
    geometry_msgs
    std_msgs
)

generate_dynamic_reconfigure_options(
    cfg/ObjectsClassifier.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS message_runtime roscpp
    INCLUDE_DIRS include
    #LIBRARIES ${PROJECT_NAME}
)

set(NLOHMANN_JSON_HEADER_LOCATION "$ENV{UTCOUPE_WORKSPACE}/libs/json/single_include/")

include_directories(
    include
    ${NLOHMANN_JSON_HEADER_LOCATION}
    ${catkin_INCLUDE_DIRS}
)


add_executable(objects_classifier_node
        src/objects_classifier_node.cpp src/objects_listener.cpp src/shapes.cpp src/processing_thread.cpp
        src/main_thread.cpp src/map_objects.cpp src/markers_publisher.cpp)

add_dependencies(objects_classifier_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


target_link_libraries(
    objects_classifier_node
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install (
    TARGETS objects_classifier_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY include/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(
    DIRECTORY cfg/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
)
