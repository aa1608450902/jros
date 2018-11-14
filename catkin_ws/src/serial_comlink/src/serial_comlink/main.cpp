#include <ros/ros.h>
#include <serial_comlink/serial_comlink_node.h>
#include <thread>
#include <boost/thread.hpp>
#include <laserd/Point32Plus.h>
#include <sstream>
#include <iostream>
#include <chrono>
#include <deque>
using namespace std;

int main(int argc, char* argv[]) {
	ROS_INFO("start serial comlink node ...");

//	ros::init(argc, argv, "serial_comlink_node");
	ros::init(argc, argv, "SerialCom");

	serial_comlink::SerialComlinkNode serialComlinkNode;

	serialComlinkNode.main();

	ROS_INFO("Terminate SerialComlinkNode");

	return 0;
}