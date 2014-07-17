cmake_minimum_required(VERSION 2.8.3)
project(flir_gige)

set(CMAKE_BUILD_TYPE Debug)

add_definitions("-Wall -std=c++11")

# Set library paths for pleora ebus sdk on 64-bit linux
# For more information, see cmake/Findebus.cmake
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    # Make CMake aware of the cmake folder for local FindXXX scripts.
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
    find_package(ebus REQUIRED)
else(CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "This package only supports 64-bit linux")
endif(CMAKE_SIZEOF_VOID_P EQUAL 8)

find_package(catkin REQUIRED
    COMPONENTS roscpp sensor_msgs image_transport camera_info_manager
    dynamic_reconfigure
    )
find_package(OpenCV)

# Dynamic reconfigure option, as to be placed right before catkin_package
generate_dynamic_reconfigure_options(cfg/Flir.cfg)

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES flir
    #  CATKIN_DEPENDS roscpp sensor_msgs
    #  DEPENDS system_lib
    )

include_directories(include)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EBUS_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

add_library(${PROJECT_NAME} src/gige_camera.cpp)
target_link_libraries(${PROJECT_NAME}
    ${EBUS_LIBRARIES}
    ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/flir_gige_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    )

## Add cmake target dependencies of the executable/library
add_dependencies(${PROJECT_NAME}_node
    ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

## Add gtest based cpp test }target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_flir.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
