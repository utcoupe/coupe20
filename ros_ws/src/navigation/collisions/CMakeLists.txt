cmake_minimum_required(VERSION 3.0.2) # kinetic requirement
project(collisions)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    ard_asserv
    game_manager
    geometry_msgs
    geometry_tools
    genmsg
    navigator
    objects_classifier
    roscpp
    roslib
    rospy
    static_map
    std_msgs
    tf2
    tf2_ros
    visualization_msgs
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

add_library(${PROJECT_NAME}_warnings INTERFACE)
add_library(${PROJECT_NAME}_options INTERFACE)

target_compile_features(${PROJECT_NAME}_options INTERFACE cxx_std_14)
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
    # -Wall
    # -Wextra
    # -Wpedantic
    # -Wshadow
    # -Wnon-virtual-dtor
    
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

add_library(${PROJECT_NAME}_shapes
    src/collisions/shapes/circle.cpp
    src/collisions/shapes/rectangle.cpp
    src/collisions/shapes/segment.cpp
)
target_link_libraries(${PROJECT_NAME}_shapes PRIVATE
    ${PROJECT_NAME}_options
    ${PROJECT_NAME}_warnings
    ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_shapes
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

set(SOURCE_FILES_COLLISIONS_NODE
    src/collisions/collisions_node_class.cpp
    src/collisions/collisions_subscriptions.cpp
    src/collisions/markers_publisher.cpp
    src/collisions/obstacle_velocity.cpp
    src/collisions/obstacle.cpp
    src/collisions/obstacles_stack.cpp
    src/collisions/robot.cpp
    src/collisions/engine/engine.cpp
    src/collisions/engine/path_check_zone.cpp
    src/collisions/engine/velocity_check_zone.cpp
)

add_library(${PROJECT_NAME}  ${SOURCE_FILES_COLLISIONS_NODE})
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${PROJECT_NAME}_options
        ${PROJECT_NAME}_warnings
        ${catkin_LIBRARIES}
    PUBLIC
        ${PROJECT_NAME}_shapes
)
add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_executable(${PROJECT_NAME}_node src/collisions_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE
    ${PROJECT_NAME}_options
    ${PROJECT_NAME}_warnings
    ${PROJECT_NAME}_shapes
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS ${PROJECT_NAME}_node
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

option(UTCOUPE_RUN_TESTS "Enable tests compilation and run" FALSE)

if (UTCOUPE_RUN_TESTS)
    ENABLE_TESTING ()
    SET(CATCH_FOLDER "$ENV{UTCOUPE_WORKSPACE}/libs/catch2")
    
    add_executable(${PROJECT_NAME}_unit_tests
        tests/test_shapes.cpp
        tests/main_unit_tests.cpp
    )
    target_include_directories(${PROJECT_NAME}_unit_tests PRIVATE
        ${CATCH_FOLDER}
    )
    target_link_libraries(${PROJECT_NAME}_unit_tests PRIVATE
        ${PROJECT_NAME}_options
        ${PROJECT_NAME}_warnings
        ${PROJECT_NAME}_shapes
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
    )
    add_dependencies(${PROJECT_NAME}_unit_tests
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )
    add_custom_command(
        TARGET ${PROJECT_NAME}_unit_tests
        POST_BUILD
        COMMAND ${PROJECT_NAME}_unit_tests
        COMMENT "Running ${PROJECT_NAME}_unit_tests..."
    )
endif ()
