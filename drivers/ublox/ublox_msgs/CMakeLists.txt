cmake_minimum_required(VERSION 2.8.3)
project(ublox_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation ublox_serialization std_msgs sensor_msgs)

add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime ublox_serialization std_msgs sensor_msgs)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${ublox_serialization_INCLUDE_DIRS})

add_library(ublox_msgs src/ublox_msgs.cpp)
