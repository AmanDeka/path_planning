cmake_minimum_required(VERSION 3.10)

project(PathPlanning)

find_package(ompl CONFIG REQUIRED)

add_executable(PathPlanning ompl_rrt.cpp)

target_link_libraries(PathPlanning PRIVATE ompl)
