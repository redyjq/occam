cmake_minimum_required(VERSION 2.8)
project(occam)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  #cv_bridge
)

set(OpenCV_DIR "/home/rpal/opencv/build")
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}/bin")

add_executable(occam src/read_point_cloud.cc)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include(FindOpenCV.cmake)
include_directories(${OpenCV2_INCLUDE_DIRS} ${OpenCV_DIR})
target_link_libraries(occam ${OpenCV_LIBS_OPT})

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(occam ${PCL_LIBRARIES})

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})

find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
target_link_libraries(occam ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/indigosdk-2.0.15/include)
add_subdirectory(indigosdk-2.0.15)
target_link_libraries(occam indigo)

add_executable(point_cloud_from_disparity src/point_cloud_from_disparity.cc)
target_link_libraries(point_cloud_from_disparity indigo ${PCL_LIBRARIES} ${OPENGL_LIBRARIES})

target_link_libraries(occam ${catkin_LIBRARIES})

#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)