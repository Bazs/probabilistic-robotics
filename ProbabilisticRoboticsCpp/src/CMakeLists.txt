cmake_minimum_required (VERSION 3.5.1)

project (ProbabilisticRobotics)

# The version number
set (ProbabilisticRobotics_VERSION_MAJOR 0)
set (ProbabilisticRobotics_VERSION_MINOR 1)
configure_file (
  "${PROJECT_SOURCE_DIR}/ProbRobConfig.h.in"
  "${PROJECT_BINARY_DIR}/ProbRobConfig.h"
)

# C++ compiler
SET (CMAKE_CXX_STANDARD 14) 

# Find third party libraries
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

SET(COMMON_INCLUDES "${CMAKE_SOURCE_DIR}/Common")
SET(EIGEN_INCLUDES "${EIGEN3_INCLUDE_DIR}")

INCLUDE_DIRECTORIES(KalmanFilter Solutions Common)
add_subdirectory (KalmanFilter)
add_subdirectory (Solutions)
