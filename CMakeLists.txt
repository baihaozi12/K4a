# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(robot_kinect)
set(CMAKE_INSTALL_BINDIR `pwd`)
add_executable(robot_kinect main.cpp lenz_camera.h generate_ply.cpp generate_ply.h io_utils.hpp)
#set_property(TARGET rs-save-to-disk PROPERTY CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(OpenCV_DIR "/usr/local/share/opencv4")

find_package(OpenSSL REQUIRED)
#find_package(CURL CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
link_directories( ${OpenCV_LIB_DIR})
#include_directories(PRVIATE  ${OpenCV_INCLUDE_DIRS} includes /usr/local/include/ )
target_link_libraries(robot_kinect PRIVATE ${OpenCV_LIBS} OpenSSL::SSL OpenSSL::Crypto    -lcrypto  /usr/local/lib/libk4a.so /usr/local/lib/libk4arecord.so /usr/lib/x86_64-linux-gnu/libdepthengine.so.2.0 /usr/lib/x86_64-linux-gnu/libdepthengine.so.1.0)
set_target_properties (robot_kinect PROPERTIES FOLDER "Examples")
install(TARGETS robot_kinect RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
