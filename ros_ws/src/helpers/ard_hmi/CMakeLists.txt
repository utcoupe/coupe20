cmake_minimum_required(VERSION 2.8.3)
project(drivers_ard_hmi)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
)

add_message_files(
  FILES
  SetTeams.msg
  SetStrategies.msg
  HMIEvent.msg
  ROSEvent.msg
)

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)