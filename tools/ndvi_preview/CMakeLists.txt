cmake_minimum_required(VERSION 2.8.3)
project(ndvi_preview)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge image_transport roscpp sensor_msgs std_msgs pcl_ros
    pcl_conversions laser_assembler message_generation
    )

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

add_service_files(FILES
    SaveToPcd.srv
    LoadFromPcd.srv
    )

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    )

add_executable(${PROJECT_NAME} src/ndvi_preview_node.cpp)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

add_executable(ndvi_cloud_node src/ndvi_cloud_node.cpp)
target_link_libraries(ndvi_cloud_node ${catkin_LIBRARIES})

add_executable(ndvi_mosaic_node src/ndvi_mosaic_node.cpp)
target_link_libraries(ndvi_mosaic_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
    )

add_dependencies(${PROJECT_NAME}
    ${PROJECT_NAME}_gencpp
    ${catkin_EXPORTED_TARGETS}
    )
