cmake_minimum_required (VERSION 3.5.1)
project (ProbabilisticRobotics)
# The version number.
set (ProbabilisticRobotics_VERSION_MAJOR 0)
set (ProbabilisticRobotics_VERSION_MINOR 1)

configure_file (
  "${PROJECT_SOURCE_DIR}/ProbRobConfig.h.in"
  "${PROJECT_BINARY_DIR}/ProbRobConfig.h"
)
  
include_directories("${PROJECT_BINARY_DIR}")
include_directories ("${PROJECT_SOURCE_DIR}/KalmanFilter")
include_directories ("${PROJECT_SOURCE_DIR}/Solutions")
add_subdirectory (KalmanFilter)
add_subdirectory (Solutions)
