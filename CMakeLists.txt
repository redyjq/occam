cmake_minimum_required(VERSION 2.8)
project(occam)

set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}/bin")

add_executable(occam src/read_point_cloud.cc)

include(FindOpenCV.cmake)
include_directories(${OpenCV2_INCLUDE_DIRS} ${OpenCV_DIR})
target_link_libraries(occam ${OpenCV_LIBS_OPT})

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(occam ${PCL_LIBRARIES})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/indigosdk-2.0.34/include)
add_subdirectory(indigosdk-2.0.34)
target_link_libraries(occam indigo)

add_executable(point_cloud_from_disparity src/point_cloud_from_disparity.cc)
target_link_libraries(point_cloud_from_disparity indigo ${PCL_LIBRARIES})