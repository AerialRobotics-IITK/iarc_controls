project(fw_att_control)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(mathlib REQUIRED)
include_directories(${mathlib_INCLUDE_DIRS})

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(ECL_lib 
    src/ecl_controller.cpp
    src/ecl_pitch_controller.cpp
    src/ecl_roll_controller.cpp
    src/ecl_wheel_controller.cpp
    src/ecl_yaw_controller.cpp
)
target_link_libraries(ECL_lib ${mathlib_LIBRARIES})

cs_install()
cs_export()