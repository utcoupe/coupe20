cmake_minimum_required(VERSION 2.8.3)
project(recognition_enemy_tracker)

find_package(catkin REQUIRED COMPONENTS
    ai_game_manager
    recognition_objects_classifier
)

catkin_package(
    CATKIN_DEPENDS
    ai_game_manager
    recognition_objects_classifier
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
    PROGRAMS
        src/enemy_tracker_node.py
        src/enemy_tracker_properties.py
        src/enemy_tracker_tracker.py
        src/libtools.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY src/Data
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    PATTERN "*.pyc" EXCLUDE
)

install(
    DIRECTORY def/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/def
)
