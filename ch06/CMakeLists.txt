CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(ch6)

# SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS "-std=c++14 -O3")
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# OpenCV
SET(OpenCV_DIR "~/App/opencv/opencv-3.4/build")
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# Ceres
# SET(Ceres_DIR "${PROJECT_SOURCE_DIR}/../3rdparty/ceres-solver/build/installed")
FIND_PACKAGE(Ceres REQUIRED PATHS "${PROJECT_SOURCE_DIR}/../3rdparty/ceres-solver/build/installed")
INCLUDE_DIRECTORIES(${Ceres_INCLUDE_DIRS})

# g2o
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../3rdparty/g2o/cmake_modules)
SET(G2O_ROOT ${PROJECT_SOURCE_DIR}/../3rdparty/g2o/build/installed)
SET(G2O_INCLUDE ${PROJECT_SOURCE_DIR}/../3rdparty/g2o/build/installed/include)
FIND_PACKAGE(G2O REQUIRED)
INCLUDE_DIRECTORIES(${G2O_ROOT})
INCLUDE_DIRECTORIES(${G2O_INCLUDE})

# Eigen
INCLUDE_DIRECTORIES("/usr/include/eigen3")

# gaussNewton
ADD_EXECUTABLE(gaussNewton src/gaussNewton.cpp)
TARGET_LINK_LIBRARIES(gaussNewton ${OpenCV_LIBS})

# ceresCurveFitting
ADD_EXECUTABLE(ceresCurveFitting src/ceresCurveFitting.cpp)
TARGET_LINK_LIBRARIES(ceresCurveFitting ${OpenCV_LIBS} ${CERES_LIBRARIES})

# g2oCurveFitting
ADD_EXECUTABLE(g2oCurveFitting src/g2oCurveFitting.cpp)
TARGET_LINK_LIBRARIES(g2oCurveFitting ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})
