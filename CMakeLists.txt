cmake_minimum_required(VERSION 2.8.3)
project(occam)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  # cv_bridge
  beam_joy
  dynamic_reconfigure
  pcl_ros
  # OpenCV
)

# set(OpenCV_DIR "/home/rpal/opencv/build")
# set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin")
# set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}/bin")

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Msg1.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

generate_dynamic_reconfigure_options(cfg/Occam.cfg)

## Declare a catkin package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beam_joy
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(main src/read_point_cloud.cc)

# include(FindOpenCV.cmake)
# include_directories(${OpenCV2_INCLUDE_DIRS} ${OpenCV_DIR})
# target_link_libraries(main ${OpenCV_LIBS_OPT})

# find_package(PCL 1.7 REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# target_link_libraries(main ${PCL_LIBRARIES})

# find_package(GLUT REQUIRED)
# include_directories(${GLUT_INCLUDE_DIRS})
# link_directories(${GLUT_LIBRARY_DIRS})
# add_definitions(${GLUT_DEFINITIONS})

# find_package(OpenGL REQUIRED)
# include_directories(${OpenGL_INCLUDE_DIRS})
# link_directories(${OpenGL_LIBRARY_DIRS})
# add_definitions(${OpenGL_DEFINITIONS})
# target_link_libraries(main ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

# include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/indigosdk-2.0.15/include)
add_subdirectory(indigosdk-2.0.15)
target_link_libraries(main indigo)

find_package(OpenCV)
# include_directories(${OpenCV_INCLUDE_DIRS})
# target_link_libraries(main ${OpenCV_LIBRARIES})
include_directories(${OpenCV2_INCLUDE_DIRS} ${OpenCV_DIR})
target_link_libraries(main indigo ${OpenCV_LIBS_OPT})

# add_executable(point_cloud_from_disparity src/point_cloud_from_disparity.cc)
# target_link_libraries(point_cloud_from_disparity indigo ${PCL_LIBRARIES} ${OPENGL_LIBRARIES})

target_link_libraries(main ${catkin_LIBRARIES})
add_dependencies(main ${PROJECT_NAME}_gencfg)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cv_bridge)
add_subdirectory(cv_bridge)
target_link_libraries(main cv_bridge)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)