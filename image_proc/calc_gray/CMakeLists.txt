cmake_minimum_required(VERSION 2.8)

project(calc_gray)

################# self define #################
set(MY_PROJ /home/u01/my_proj)
set(PROJECT_ROOT ${MY_PROJ}/calc_gray)
set(SOURCE_ROOT ${PROJECT_ROOT}/src)
#set(CMAKE_BINARY_DIR ${PROJECT_ROOT}/build)

################# cmake define #################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_ROOT}/build)

#find_package( OpenCV 3 REQUIRED )

################# settings #################
include_directories(
	/usr/include
    /usr/local/include
	${SOURCE_ROOT}
)

add_executable(calc_gray
	src/main.cpp
	src/CalcGray.cpp
)

link_directories(
	/usr/lib
	/usr/local/lib
)

target_link_libraries(calc_gray
	opencv_core opencv_highgui opencv_imgproc
)
