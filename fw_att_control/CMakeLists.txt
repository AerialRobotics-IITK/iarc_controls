project(fw_att_control)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(mathlib REQUIRED)
include_directories(${mathlib_INCLUDE_DIRS})

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(${PROJECT_NAME}_ecl_controller src/ecl_controller.cpp)
target_link_libraries(${PROJECT_NAME}_ecl_controller ${mathlib_LIBRARIES})

cs_install()
cs_export()