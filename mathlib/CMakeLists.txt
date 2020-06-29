project(mathlib)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)


cs_install()
cs_export()