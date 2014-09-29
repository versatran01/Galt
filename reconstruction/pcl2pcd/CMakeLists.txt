cmake_minimum_required(VERSION 2.8.3)
project(pcl2pcd)

find_package(catkin REQUIRED COMPONENTS
    roscpp laser_assembler message_generation
    pcl_conversions pcl_ros
    )

add_service_files(FILES
    SaveToPcd.srv
    ResetTime.srv
    )

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES pcl2pcd
    #  CATKIN_DEPENDS laser_assembler
    #  DEPENDS system_lib
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

add_executable(${PROJECT_NAME}_srv
    src/pcl2pcd_srv.cpp
    src/pcl2pcd_srv_main.cpp
    )
target_link_libraries(${PROJECT_NAME}_srv
    ${catkin_LIBRARIES}
    )

add_executable(${PROJECT_NAME}_rviz
    src/pcl2pcd_rviz.cpp
    src/pcl2pcd_rviz_main.cpp
    )
target_link_libraries(${PROJECT_NAME}_rviz
    ${catkin_LIBRARIES}
    )


add_dependencies(${PROJECT_NAME}_srv
    ${PROJECT_NAME}_gencpp
    )
