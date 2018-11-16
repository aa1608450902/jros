cmake_minimum_required(VERSION 2.8.3)

project(laserd)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

set(EXECUTABLE_OUTPUT_PATH /home/u01/catkin_ws/devel/lib/laserd)

include_directories(
    /usr/include
    /opt/ros/indigo/include
    /home/u01/catkin_ws/devel/include
    src
    include
)

link_directories(
#    /usr/lib
    /opt/ros/indigo/lib
#    /usr/local/lib
)

add_executable(laserd_node
    src/laserd_node.cpp
    src/LaserdNode.cpp
    src/laserd.cpp
)

target_link_libraries(laserd_node
    roscpp
    rosconsole
    roscpp_serialization
    opencv_core opencv_highgui opencv_imgproc opencv_imgcodecs
    glog
    /opt/ros/indigo/lib/libcv_bridge.so
)
