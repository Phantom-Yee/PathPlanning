cmake_minimum_required(VERSION 3.10)
project(PathPlanningLib)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(pathplanning SHARED
    path_planning.cpp
)

target_include_directories(pathplanning PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
