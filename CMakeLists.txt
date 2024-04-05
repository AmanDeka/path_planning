cmake_minimum_required(VERSION 3.10)

project(PathPlanning)

set(DCMAKE_TOOLCHAIN_FILE "D:/vcpkg/scripts/buildsystems/vcpkg.cmake")
set(CMAKE_TOOLCHAIN_FILE "D:/vcpkg/scripts/buildsystems/vcpkg.cmake")

add_executable(PathPlanning ompl_rrt.cpp)

find_package(ompl CONFIG REQUIRED)


target_link_libraries(PathPlanning PRIVATE ompl)
