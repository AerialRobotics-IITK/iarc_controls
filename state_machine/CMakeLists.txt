cmake_minimum_required(VERSION 2.8.3)
project(state_machine)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

set(ASM agent_state_machine)

cs_add_library(${ASM}_lib
    src/${ASM}/${ASM}_base.cpp
    src/${ASM}/${ASM}.cpp
)
target_link_libraries(${ASM}_lib ${catkin_LIBRARIES})

cs_add_executable(${ASM}_node src/${ASM}/${ASM}_node.cpp)
target_link_libraries(${ASM}_node ${ASM}_lib)
