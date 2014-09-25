cmake_minimum_required(VERSION 2.8.3)
project(graph_recorder)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt4 COMPONENTS QtCore REQUIRED)
include(${QT_USE_FILE})

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  image_geometry
  kr_math
  nav_msgs
  message_filters
  roscpp
  sensor_msgs
  stereo_vo
  visualization_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Qt4_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
)
include_directories(include)

set(${PROJECT_NAME}_SOURCES
  src/graph_recorder_node.cpp
  src/node.cpp
)

set(${PROJECT_NAME}_HEADERS
  include/graph_recorder/node.hpp
)

# invoke MOC and UI
QT4_WRAP_CPP(${PROJECT_NAME}_MOCSrcs ${${PROJECT_NAME}_HEADERS})

set(PROJECT_SOURCE_FILES
  ${${PROJECT_NAME}_SOURCES}
  ${${PROJECT_NAME}_MOCSrcs}
)


add_definitions("-Wall -Wunused -std=c++11")
add_executable(${PROJECT_NAME}
  ${PROJECT_SOURCE_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
