cmake_minimum_required(VERSION 2.8.3)
project(yield_map)

add_definitions(-Wall)
find_package(OpenCV)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(catkin REQUIRED COMPONENTS
    roscpp sensor_msgs pcl_conversions pcl_ros message_generation)

add_service_files(FILES SaveToPcd.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(yield_map_node src/yield_map_node.cpp)
target_link_libraries(yield_map_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(generate_yield_map src/generate_yield_map.cpp)
target_link_libraries(generate_yield_map ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(yield_map_node
    ${PROJECT_NAME}_gencpp
    ${catkin_EXPORTED_TARGETS}
    )
