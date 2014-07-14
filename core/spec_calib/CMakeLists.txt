cmake_minimum_required(VERSION 2.8.3)
project(spec_calib)

# include QT
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5Widgets REQUIRED)

# find OpenCV separately
find_package(OpenCV REQUIRED)

# ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  image_transport
  cv_bridge
  circle_tracker
  ocean_optics
  monocular_pose_estimator
  kr_math
)

catkin_package()

set(${PROJECT_NAME}_SOURCES
  src/main.cpp
  src/calibrate_line.cpp
  src/qt/mainwindow.cpp
  src/qt/posecalibrationview.cpp
  src/qt/posecalibrator.cpp
  src/qt/cvimagewidget.cpp
)
set(${PROJECT_NAME}_HEADERS
  src/qt/mainwindow.h
  src/qt/posecalibrationview.h
  src/qt/posecalibrator.h
  src/qt/cvimagewidget.h
)
set(${PROJECT_NAME}_FORMS
  forms/mainwindow.ui
  forms/posecalibrationview.ui
)

# invoke MOC and UI
QT5_WRAP_CPP(${PROJECT_NAME}_MOCSrcs ${${PROJECT_NAME}_HEADERS})
QT5_WRAP_UI(${PROJECT_NAME}_UISrcs ${${PROJECT_NAME}_FORMS})

# QT Declarations
include_directories(
  src
  src/qt
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Qt5Widgets}_INCLUDE_DIRS
)
# defintions
add_definitions(${Qt5Widgets_DEFINITIONS})
# compiler flags for QT5
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

# include headers for ROS
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# additional warnings and cpp11
add_definitions("-Wall -Wunused -std=c++11")

# build node
add_executable(${PROJECT_NAME}
  ${${PROJECT_NAME}_SOURCES}
  ${${PROJECT_NAME}_MOCSrcs}
  ${${PROJECT_NAME}_UISrcs}
)

# link QT and ROS
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# depends on messages from spectrometer
add_dependencies(${PROJECT_NAME} ${ocean_optics_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${circle_tracker_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${monocular_pose_estimator_EXPORTED_TARGETS})
