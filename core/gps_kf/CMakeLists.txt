cmake_minimum_required(VERSION 2.8.3)
project(gps_kf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  kr_math
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(${PROJECT_NAME}
  src/node.cpp
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
