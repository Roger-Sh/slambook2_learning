cmake_minimum_required(VERSION 3.10)
project(SLAMBOOK2)

# 设置环境支持C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(chapter_demo/ch02)
add_subdirectory(chapter_demo/ch03)
add_subdirectory(chapter_demo/ch04)
add_subdirectory(chapter_demo/ch05)
add_subdirectory(chapter_demo/ch06)
add_subdirectory(chapter_demo/ch07)
add_subdirectory(chapter_demo/ch08)