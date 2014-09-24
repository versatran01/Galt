cmake_minimum_required(VERSION 2.8.3)
project(stereo_vo)

add_definitions(-Wall -std=c++11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
    roscpp nodelet image_transport message_filters
    sensor_msgs visualization_msgs cv_bridge image_geometry
    dynamic_reconfigure eigen_conversions message_generation
    kr_math rviz_helper std_msgs geometry_msgs sophus
    )

find_package(OpenCV)
#find_package(gtsam)
#find_package(G2O REQUIRED)

generate_dynamic_reconfigure_options(cfg/StereoVoDyn.cfg)
add_message_files(DIRECTORY msg
    FILES FeatureMsg.msg FeaturesStamped.msg StereoFeaturesStamped.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES stereo_vo
    #  CATKIN_DEPENDS roscpp
    #  DEPENDS system_lib
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    #${GTSAM_INCLUDE_DIRS}
    #${G2O_INCLUDE_DIR}
    #/usr/include/suitesparse # g2o may require this
    )

add_library(${PROJECT_NAME}
    src/stereo_vo.cpp
    src/frame.cpp
    src/detector.cpp
    src/feature.cpp
    src/utils.cpp
#    src/tracker.cpp
#    src/point3d.cpp
    #src/optimizer.cpp
    )
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    #${GTSAM_LIBRARIES}
    #${G2O_CORE_LIBRARY}
    #${G2O_STUFF_LIBRARY}
    #${G2O_SOLVER_CHOLMOD}
    #${G2O_SOLVER_CSPARSE}
    #${G2O_SOLVER_DENSE}
    #${G2O_SOLVER_PCG}
    #${G2O_TYPES_SBA}
    #cxsparse
    #cholmod
    )

add_executable(${PROJECT_NAME}_node
    src/stereo_vo_main.cpp
    src/stereo_vo_node.cpp
    )
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    )

add_executable(test_detector
    test/test_detector.cpp
    )
target_link_libraries(test_detector
    ${OpenCV_LIBRARIES}
    ${PROJECT_NAME}
    )

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    ${PROJECT_NAME}_gencpp
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
