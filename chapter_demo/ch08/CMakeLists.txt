CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(ch08)

# SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 使用SSE指令集
ADD_DEFINITIONS("-DENABLE_SSE")
SET(CMAKE_CXX_FLAGS "-std=c++11 ${SSE_FLAGS} -g -O3 -march=native")

# OpenCV
# FIND_PACKAGE(OpenCV 3.2.0 REQUIRED)
find_package(OpenCV 4.0 REQUIRED
    PATHS "${PROJECT_SOURCE_DIR}/../../3rdparty/opencv-4.0.0/build/installed"
    NO_DEFAULT_PATH
)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# Sophus
SET(Sophus_DIR "${PROJECT_SOURCE_DIR}/../../3rdparty/Sophus/build/installed")
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# Pangolin
set(Pangolin_DIR "${PROJECT_SOURCE_DIR}/../../3rdparty/Pangolin/build/installed")
find_package(Pangolin REQUIRED)

# G2O
LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../../3rdparty/g2o/cmake_modules")
SET(G2O_ROOT ${PROJECT_SOURCE_DIR}/../../3rdparty/g2o/build/installed)
SET(G2O_INCLUDE ${PROJECT_SOURCE_DIR}/../../3rdparty/g2o/build/installed/include)
FIND_PACKAGE(G2O REQUIRED)
INCLUDE_DIRECTORIES(${G2O_INCLUDE})

# Eigen
include_directories("/usr/include/eigen3/")

# optical_flow
add_executable(optical_flow src/optical_flow.cpp)
target_link_libraries(optical_flow ${OpenCV_LIBS} Sophus::Sophus)

# # direct_method
# add_executable(direct_method src/direct_method.cpp)
# target_link_libraries(direct_method
# ${OpenCV_LIBS}
# ${Pangolin_LIBRARIES}
# Sophus::Sophus)
