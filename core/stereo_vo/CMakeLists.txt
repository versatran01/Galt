cmake_minimum_required(VERSION 2.8.3)
project(stereo_vo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(OpenCV)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stereo_vo
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
