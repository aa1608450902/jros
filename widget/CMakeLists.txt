cmake_minimum_required(VERSION 3.1.0)

project(widget)

set(ENV{CMAKE_PREFIX_PATH} "/home/aiyo/Qt5.11.2/5.11.2/gcc_64/lib/cmake:$ENV{CMAKE_PREFIX_PATH}")

find_package(Qt5Core)

find_package(Qt5Widgets)

add_executable(widget
	main.cpp
	TopWnd.h TopWnd.cpp moc_TopWnd.cpp
	CtrlWnd.h CtrlWnd.cpp moc_CtrlWnd.cpp
	ViewWnd.h ViewWnd.cpp moc_ViewWnd.cpp
)

target_link_libraries(widget Qt5::Core Qt5::Widgets)