cmake_minimum_required(VERSION 2.8.0)

project(serial_linker)

set(EXECUTABLE_OUTPUT_PATH /home/u01/catkin_ws/devel/lib/serial_linker)

add_definitions(-std=c++11)

include_directories(
    /opt/ros/indigo/include
    /home/u01/catkin_ws/devel/include
    )

link_directories(
    /opt/ros/indigo/lib
    )

add_executable(serial_linker_node src/main.cpp)

target_link_libraries(serial_linker_node
    roscpp rosconsole roscpp_serialization pthread
    )