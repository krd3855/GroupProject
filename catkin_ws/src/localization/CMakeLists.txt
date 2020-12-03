cmake_minimum_required(VERSION 2.8.3)
project(aruco_loc)

## Compile as C++11, supported in ROS Kinetic and newer

set(CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
    roscpp
    image_transport
    cv_bridge
    sensor_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    message_generation
    tf
    image_geometry
)

## System dependencies are found with CMake's conventions

find_package(OpenCV REQUIRED)
link_directories(${OpenCV_LIBRARY_DIRS})

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#  MarkerMeasurement.msg
#  MarkerMeasurementArray.msg
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  geometry_msgs
#)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${aruco_INCLUDE_DIRS})

set(SOURCES src/aruco_loc_node.cpp src/aruco_detection.cpp src/thread.cpp)

add_executable(aruco_loc ${SOURCES})

target_link_libraries(aruco_loc ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${aruco_LIBS} stdc++fs)

