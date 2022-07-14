CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(VO1)

SET(CMAKE_BUILD_TYPE "Release")
ADD_DEFINITIONS("-DENABLE_SSE") # 使用SSE指令集
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${SSE_FLAGS} -msse4")
SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


#### LINK

# OpenCV
SET(OpenCV_DIR "~/App/opencv/opencv-3.4/build")
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

#G2O
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../3rdparty/g2o/cmake_modules)
SET(G2O_ROOT ${PROJECT_SOURCE_DIR}/../3rdparty/g2o/build/installed)
SET(G2O_INCLUDE ${PROJECT_SOURCE_DIR}/../3rdparty/g2o/build/installed/include)
find_package(G2O REQUIRED)
INCLUDE_DIRECTORIES(${G2O_ROOT})
INCLUDE_DIRECTORIES(${G2O_INCLUDE})

# Sophus
SET(Sophus_DIR "${PROJECT_SOURCE_DIR}/../3rdparty/Sophus/build/installed")
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3/")

#### BUILD

# orb_cv
add_executable(orb_cv src/orb_cv.cpp)
target_link_libraries(orb_cv ${OpenCV_LIBS})

# orb_self
add_executable(orb_self src/orb_self.cpp)
target_link_libraries(orb_self ${OpenCV_LIBS})

# pose_estimation_2d2d
# add_executable( pose_estimation_2d2d pose_estimation_2d2d.cpp extra.cpp ) # use this if in OpenCV2 
add_executable(pose_estimation_2d2d src/pose_estimation_2d2d.cpp)
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBS})

# triangulation
# # add_executable( triangulation triangulation.cpp extra.cpp) # use this if in opencv2
add_executable(triangulation src/triangulation.cpp)
target_link_libraries(triangulation ${OpenCV_LIBS})

# pose_estimation_3d2d
add_executable(pose_estimation_3d2d src/pose_estimation_3d2d.cpp)
target_link_libraries(pose_estimation_3d2d
    ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY}
    ${OpenCV_LIBS}
    Sophus::Sophus)

# pose_estimation_3d3d
add_executable(pose_estimation_3d3d src/pose_estimation_3d3d.cpp)
target_link_libraries(pose_estimation_3d3d
    ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY}
    ${OpenCV_LIBS}
    Sophus::Sophus)
