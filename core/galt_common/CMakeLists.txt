cmake_minimum_required(VERSION 2.8.3)
project(galt_common)
find_package(catkin REQUIRED)

find_package(Eigen3 REQUIRED)

# Export Eigen include directories as well
catkin_package(INCLUDE_DIRS include ${Eigen3_INCLUDE_DIR}
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS roscpp
               CFG_EXTRAS extras.cmake)

include_directories(include ${Eigen3_INCLUDE_DIR})
add_library(${PROJECT_NAME} src/galt_common.cpp)
