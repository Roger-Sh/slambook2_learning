# set for ubuntu 20
option(USE_UBUNTU_20 "Set to ON if you are using Ubuntu 20.04" OFF)

# Pangolin
set(Pangolin_DIR "${PROJECT_SOURCE_DIR}/../../3rdparty/Pangolin/build/installed")
find_package(Pangolin REQUIRED)

# # Sophus
# set(Sophus_DIR "${PROJECT_SOURCE_DIR}/../../3rdparty/Sophus/build/installed")
# find_package(Sophus REQUIRED)

# fmt for ubuntu 20
if(USE_UBUNTU_20)
    message("You are using Ubuntu 20.04, fmt::fmt will be linked")
    find_package(fmt REQUIRED)
    set(FMT_LIBRARIES fmt::fmt)
endif()

include_directories(${Pangolin_INCLUDE_DIRS})
# include_directories(${Sophus_INCLUDE_DIRS})
add_executable(trajectoryError trajectoryError.cpp)
target_link_libraries(trajectoryError ${Pangolin_LIBRARIES} ${FMT_LIBRARIES} Sophus::Sophus)
