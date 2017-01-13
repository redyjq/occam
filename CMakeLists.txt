cmake_minimum_required(VERSION 2.8.3)
project(occam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  beam_joy
  dynamic_reconfigure
  cv_bridge
)

generate_dynamic_reconfigure_options(cfg/Occam.cfg)

catkin_package()

add_executable(occam src/read_point_cloud.cc)
add_dependencies(occam ${PROJECT_NAME}_gencfg)
add_subdirectory(indigosdk-2.0.15)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(occam ${PCL_LIBRARIES})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV2_INCLUDE_DIRS} ${OpenCV_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/indigosdk-2.0.15/include
)

target_link_libraries(occam 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS_OPT}
  indigo
)

set(CMAKE_BUILD_TYPE Release)