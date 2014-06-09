cmake_minimum_required(VERSION 2.8.3)
project(bluefox2)

find_package(catkin REQUIRED COMPONENTS
    roscpp sensor_msgs image_transport camera_info_manager rostest)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

set(MVIMPACT_DIR "${PROJECT_SOURCE_DIR}/mvIMPACT") # 2.7.2
include_directories(${MVIMPACT_DIR})

add_executable(bluefox2 src/camera_node.cpp src/camera.cpp)
target_link_libraries(bluefox2
    ${catkin_LIBRARIES}
    ${MVIMPACT_DIR}/lib/x86_64/libmvDeviceManager.so)

# Unit test uses gtest, but needs rostest to create a ROS environment.
# Hence, it must be created as a normal executable, not using
# catkin_add_gtest() which runs an additional test without rostest.
add_executable(rtest_mono tests/rtest_mono.cpp)
target_link_libraries(rtest_mono ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
add_rostest(tests/rtest_mono.test)
