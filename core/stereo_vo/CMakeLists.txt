cmake_minimum_required(VERSION 2.8.3)
project(stereo_vo)

set(CMAKE_CXX_FLAGS "-Wall -std=c++11 -fPIC")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

find_package(catkin REQUIRED COMPONENTS
    roscpp nodelet
    image_transport message_filters
    sensor_msgs visualization_msgs
    cv_bridge image_geometry
    dynamic_reconfigure
    kr_math
    eigen_conversions
)

find_package(OpenCV)
find_package(Glog)
IF (GLOG_FOUND)
  MESSAGE("Found Google Log header in: ${GLOG_INCLUDE_DIRS}")
  MESSAGE("Google Log library: ${GLOG_LIBRARY}")
ELSE (GLOG_FOUND)
  MESSAGE(FATAL_ERROR "Can't find Google Log.")
ENDIF (GLOG_FOUND)

generate_dynamic_reconfigure_options(
    cfg/StereoVoDyn.cfg
)

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
    ${GLOG_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/stereo_vo.cc
    src/feature_detector.cc
    src/utils.cc
    src/feature.cc
#    src/ceres_bundler.cc
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ceres
    ${GLOG_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
    src/main.cc
    src/stereo_vo_node.cc
)
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
)

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    ${PROJECT_NAME}_gencpp
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
 )
