cmake_minimum_required(VERSION 2.8.3)
project(galt_attitude_eskf)
find_package(catkin REQUIRED COMPONENTS roscpp tf geometry_msgs sensor_msgs
                                        message_filters galt_common message_generation
                                        eigen_conversions kr_math)

# generate messages
add_service_files(FILES BeginCalibration.srv)
add_message_files(FILES Status.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

# include boost
find_package(Boost REQUIRED)
link_directories(${Boost_LIBRARY_DIR})
include_directories(${Boost_INCLUDE_DIR})

# include headers
include_directories(${galt_common_INCLUDE_DIRS} ${kr_math_INCLUDE_DIRS})

# build shared library
add_library(AttitudeESKF src/AttitudeESKF.cpp)

# build main filter
add_executable(${PROJECT_NAME} src/galt_attitude_eskf.cpp)

target_link_libraries(${PROJECT_NAME} AttitudeESKF
                                      ${roscpp_LIBRARIES}
                                      ${galt_common_LIBRARIES}
                                      ${tf_LIBRARIES}
                                      ${eigen_conversions_LIBRARIES}
                                      ${message_filters_LIBRARIES})

# ensures messages are generated before hand
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
