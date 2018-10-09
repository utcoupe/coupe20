cmake_minimum_required(VERSION 2.8.3)
project(recognition_cube)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    rospy
    tf2
    tf2_ros
    message_generation)

add_service_files(
    FILES
    Cube_localizer.srv
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
 )

catkin_package(

)

include_directories(

)

