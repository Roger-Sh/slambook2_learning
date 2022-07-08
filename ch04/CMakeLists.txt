cmake_minimum_required(VERSION 3.0)
project(useSophus)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Sophus
set(Sophus_DIR "${PROJECT_SOURCE_DIR}/../3rdparty/Sophus/build/installed")
find_package(Sophus REQUIRED)

# Eigen
include_directories("/usr/include/eigen3")

# useSophus
add_executable(useSophus useSophus.cpp)
target_link_libraries(useSophus Sophus::Sophus)

# example
add_subdirectory(example)
