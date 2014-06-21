cmake_minimum_required(VERSION 2.8.3)
project(bluefox2)

add_definitions("-Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp sensor_msgs image_transport camera_info_manager
    cv_bridge message_filters
    rostest)
find_package(OpenCV)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

set(MVIMPACT_DIR "${PROJECT_SOURCE_DIR}/mvIMPACT") # 2.7.2
include_directories(${MVIMPACT_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

# camera node
add_executable(${PROJECT_NAME}
    src/camera_node.cpp src/bluefox2.cpp src/bluefox2_ros.cpp)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${MVIMPACT_DIR}/lib/x86_64/libmvDeviceManager.so)

# stereo sync
add_executable(stereo_sync src/stereo_sync.cpp)
target_link_libraries(stereo_sync ${catkin_LIBRARIES})
target_link_libraries(stereo_sync ${OpenCV_LIBRARIES})

# Unit test uses gtest, but needs rostest to create a ROS environment.
# Hence, it must be created as a normal executable, not using
# catkin_add_gtest() which runs an additional test without rostest.
add_executable(rtest_mono
    tests/rtest_mono.cpp src/bluefox2.cpp src/bluefox2_ros.cpp)
target_link_libraries(rtest_mono
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    ${MVIMPACT_DIR}/lib/x86_64/libmvDeviceManager.so)
add_rostest(tests/rtest_mono.test)
