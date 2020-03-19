# asserv/shared/CMakeLists.txt
# Author: Gaëtan Blond
# Date : 19/03/2020

# This is intended to be included in main CMakeLists.txt.
# It defines the cmake vars :
# - UTCOUPE_ASSERV_SHARED_INCLUDE_DIR : absolute path to the include folder
# - UTCOUPE_ASSERV_SHARED_SOURCES : list of additionnal source files to compile

SET(UTCOUPE_ASSERV_SHARED_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB_RECURSE
    UTCOUPE_ASSERV_SHARED_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*
)