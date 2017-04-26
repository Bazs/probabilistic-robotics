cmake_minimum_required (VERSION 3.5.1)
project (ProbabilisticRobotics)
# The version number.
set (ProbabilisticRobotics_VERSION_MAJOR 0)
set (ProbabilisticRobotics_VERSION_MINOR 1)
SET (CMAKE_CXX_STANDARD 11) 

configure_file (
  "${PROJECT_SOURCE_DIR}/ProbRobConfig.h.in"
  "${PROJECT_BINARY_DIR}/ProbRobConfig.h"
)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories("${PROJECT_BINARY_DIR}")
include_directories ("${PROJECT_SOURCE_DIR}/KalmanFilter")
include_directories ("${PROJECT_SOURCE_DIR}/Solutions")
add_subdirectory (KalmanFilter)
add_subdirectory (Solutions)
