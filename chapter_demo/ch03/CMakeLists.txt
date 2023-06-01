cmake_minimum_required(VERSION 3.10)
project(chapter3)

set(CMAKE_CXX_FLAGS "-std=c++11")

# add sub module
add_subdirectory(useEigen)
add_subdirectory(useGeometry)
add_subdirectory(visualizeGeometry)
add_subdirectory(examples)
