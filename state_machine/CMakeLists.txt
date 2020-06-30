cmake_minimum_required(VERSION 2.8.3)
project(state_machine)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(${PROJECT_NAME}_lib
    src/state_machine_base.cpp
    src/state_machine.cpp
)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

cs_add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)
