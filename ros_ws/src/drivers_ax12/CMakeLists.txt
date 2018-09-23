cmake_minimum_required(VERSION 2.8.3)
project(drivers_ax12)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  message_runtime
  roscpp
  std_msgs
  drivers_port_finder
  ai_game_manager
  memory_definitions
  dynamixel_sdk
)

add_action_files(
  DIRECTORY action
  FILES
  Ax12Command.action
)

add_service_files(
  FILES
  SetAx12Param.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES drivers_ax12
  CATKIN_DEPENDS actionlib actionlib_msgs roscpp std_msgs drivers_port_finder ai_game_manager
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ax12_node src/ax12_node.cpp src/ax12_server.cpp src/ax12_driver.cpp)

add_dependencies(ax12_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ax12_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
    TARGETS ax12_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY include/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
