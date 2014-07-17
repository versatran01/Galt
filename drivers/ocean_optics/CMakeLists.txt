cmake_minimum_required(VERSION 2.8.3)
project(ocean_optics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  dynamic_reconfigure
)

# generate reconfigure options
generate_dynamic_reconfigure_options(cfg/OceanOptics.cfg)

# generate messages
add_message_files(FILES Spectrum.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure
)

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

message(STATUS "OceanOptics LIB DIR: ${OCEAN_OPTICS_LIB_DIR}")
message(STATUS "JavaVM LIB DIR: ${JVM_LIB_DIR}")

link_directories(
  ${JVM_LIB_DIR}
  ${OCEAN_OPTICS_LIB_DIR}
)

# add necessary definitions
add_definitions("-Wall -Wunused -DLINUX -fpic -g -std=c++11")

# point RPATH to correct location for deprecated JVM
set(CMAKE_INSTALL_RPATH ${OCEAN_OPTICS_LIB_DIR} ${JVM_LIB_DIR})

add_executable(ocean_optics src/ocean_optics_node.cpp)
target_link_libraries(ocean_optics common OmniDriver jvm)
target_link_libraries(ocean_optics ${catkin_LIBRARIES})

# ensures messages are generated before hand
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencf
)
