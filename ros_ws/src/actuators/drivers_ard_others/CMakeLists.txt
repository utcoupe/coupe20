cmake_minimum_required(VERSION 2.8.3)
project(drivers_ard_others)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    geometry_msgs
    std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    BeltRange.msg
    Move.msg
    MoveResponse.msg
    ActDigitalStates.msg
    ActPWMStates.msg
    ActServoStates.msg
    Color.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    geometry_msgs # std_msgs
)

catkin_package(
    CATKIN_DEPENDS 
    geometry_msgs 
    std_msgs
    message_runtime
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)