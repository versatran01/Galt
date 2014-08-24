cmake_minimum_required(VERSION 2.8.3)
project(flir_gige)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-unknown-pragmas")

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
    COMPONENTS roscpp sensor_msgs std_msgs
               image_transport camera_info_manager
               dynamic_reconfigure cv_bridge nodelet
    )
find_package(OpenCV)

# Dynamic reconfigure option, as to be placed right before catkin_package
generate_dynamic_reconfigure_options(
    cfg/FlirDyn.cfg
    cfg/ThermalProcDyn.cfg
    )

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

# libgige_camera
add_library(gige_camera
    src/gige_camera.cpp
    )
target_link_libraries(gige_camera
    ${EBUS_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

# libflir_gige
add_library(${PROJECT_NAME}
    src/flir_gige.cpp
    )
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    gige_camera
    )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

# node
add_executable(${PROJECT_NAME}_node
    src/flir_gige_node.cpp
    )
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    )

# thermal proc
add_library(thermal_proc
    src/thermal_proc/thermal_proc.cpp
    )
target_link_libraries(thermal_proc
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_executable(thermal_proc_node
    src/thermal_proc/thermal_proc_node.cpp
    )
target_link_libraries(thermal_proc_node
    thermal_proc
    )

# nodelet
add_library(${PROJECT_NAME}_nodelet
    src/flir_gige_nodelet.cpp
    src/thermal_proc/thermal_proc_nodelet.cpp
    )
target_link_libraries(${PROJECT_NAME}_nodelet
    ${PROJECT_NAME}
    thermal_proc
    )

## Add cmake target dependencies of the executable/library
add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    ${PROJECT_NAME}_gencpp
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
