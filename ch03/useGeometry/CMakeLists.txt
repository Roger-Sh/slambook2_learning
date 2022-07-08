cmake_minimum_required( VERSION 2.8 )
project( geometry )

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 添加Eigen头文件
# include_directories("/usr/include/eigen3")
find_package(Eigen3 REQUIRED)
include_directories(
    ${EIGEN3_INCLUDE_DIR}
)
add_definitions(${EIGEN_DEFINITIONS})

# exe
add_executable(eigenGeometry eigenGeometry.cpp)
