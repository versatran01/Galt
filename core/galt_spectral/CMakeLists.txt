cmake_minimum_required(VERSION 2.8.3)
project(galt_spectral)

# Final library will be libspectral
set(PROJECT_LIBRARY_NAME spectral)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  kr_math
)

# export dependencies
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_LIBRARY_NAME}
  CATKIN_DEPENDS kr_math
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_sources
  src/SpectrometerPose.cpp
  src/Spectrum.cpp
)

# build library
add_definitions("-Wall -Wunused -std=c++11 -fPIC")
add_library(${PROJECT_LIBRARY_NAME} SHARED
  ${${PROJECT_NAME}_sources}
)

target_link_libraries(${PROJECT_LIBRARY_NAME}
  ${catkin_LIBRARIES}
)
