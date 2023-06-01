cmake_minimum_required(VERSION 3.10)

# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O2")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Pangolin
set(Pangolin_DIR "${PROJECT_SOURCE_DIR}/../../3rdparty/Pangolin/build/installed")
find_package(Pangolin REQUIRED)

# Sophus
set(Sophus_DIR "${PROJECT_SOURCE_DIR}/../../3rdparty/Sophus/build/installed")
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# OpenCV
# set(OpenCV_DIR "~/App/opencv/opencv-3.4/build")
find_package(OpenCV 3.2.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3")

add_subdirectory(imageBasics)
add_subdirectory(stereo)
add_subdirectory(rgbd)
