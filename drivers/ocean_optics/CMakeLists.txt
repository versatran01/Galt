cmake_minimum_required(VERSION 2.8.3)
project(ocean_optics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# include OmniDriver and JDK files
include_directories(
  $ENV{JDK_6_PATH}/include/linux
  $ENV{JDK_6_PATH}/include
  $ENV{OMNIDRIVER_HOME}/include
)

# library locations
set(OCEAN_OPTICS_LIB_DIR $ENV{OOI_HOME})
set(JVM_LIB_DIR $ENV{OMNIDRIVER_HOME}/_jvm/lib/amd64/server)

message(STATUS "OO LIB DIR: ${OCEAN_OPTICS_LIB_DIR}")
message(STATUS "JVM LIB DIR: ${JVM_LIB_DIR}")

link_directories(
  ${JVM_LIB_DIR}
  ${OCEAN_OPTICS_LIB_DIR}
)

# add necessary definitions
add_definitions("-Wall -Wunused -DLINUX -fpic -g")

# point RPATH to correct location for deprecated JVM
set(CMAKE_INSTALL_RPATH ${OCEAN_OPTICS_LIB_DIR} ${JVM_LIB_DIR})

add_executable(ocean_optics src/ocean_optics_node.cpp)
target_link_libraries(ocean_optics common OmniDriver jvm)
