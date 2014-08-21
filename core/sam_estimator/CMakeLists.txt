cmake_minimum_required(VERSION 2.8.3)
project(sam_estimator)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  eigen_conversions
  kr_math
  message_filters
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

find_package(gtsam)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  include
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/sam_estimator.cpp
  src/sam_estimator_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GTSAM_LIBRARIES}
)

#add_dependencies(${PROJECT_NAME}
#  ${catkin_EXPORTED_TARGETS}
#  ${PROJECT_NAME}_gencfg
#  ${PROJECT_NAME}_gencpp
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#)
