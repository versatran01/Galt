cmake_minimum_required(VERSION 2.8.3)
project(gps_odom)

find_package(catkin REQUIRED COMPONENTS
  kr_math
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  ublox_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(${PROJECT_NAME}
  src/altitude_kf.cpp
  src/main.cpp
  src/node.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
