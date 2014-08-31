cmake_minimum_required(VERSION 2.8.3)
project(path_replay)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_definitions("-Wall -Werror -std=c++11")

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
