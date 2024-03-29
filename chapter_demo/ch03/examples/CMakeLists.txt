cmake_minimum_required(VERSION 3.10)
project(examples)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# eigen
find_package(Eigen3 REQUIRED)
include_directories(
    ${EIGEN3_INCLUDE_DIR}
)
add_definitions(${EIGEN_DEFINITIONS})

# pangolin
# set(Pangolin_DIR "${PROJECT_SOURCE_DIR}/../../../3rdparty/Pangolin/build/installed")
find_package(Pangolin REQUIRED PATHS "${PROJECT_SOURCE_DIR}/../../../3rdparty/Pangolin/build/installed" NO_DEFAULT_PATH)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(coordinateTransform coordinateTransform.cpp)
add_executable(plotTrajectory plotTrajectory.cpp)
target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})
