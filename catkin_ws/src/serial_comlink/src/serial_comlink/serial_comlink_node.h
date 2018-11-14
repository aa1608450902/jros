#pragma once

#include <map>
#include <string>

#include <ros/ros.h>
#include <laserd/Point32Plus.h>

#include <serial.h>
#include <Analyzer.h>
#include "CameraAdjuster.h"

namespace serial_comlink {

class SerialComlinkNode {
public:
	SerialComlinkNode();
	virtual ~SerialComlinkNode();

	void init();
	void main();
	void loop();

private:
	void positionCallback(const laserd::Point32Plus::ConstPtr& msg);
	std::string toHex(int n);

private:

	Analyzer _analyzer;
	CameraAdjuster _cameraAdjuster;

	enum ENV {in, out};
	const std::map<std::string, int> INS = {
			{"\xcc\x01\x01\x99", 1},
			{"\xcc\x01\x01\x06", 2},
			{"\xcc\x01\x01\x00", 3},
			{"\xcc\x01\x01\x05", 4},
			{"\xcc\x01\x01\x01", 5},
			{"\xcc\x01\x01\x11", 6},
			{"\xcc\x01\x01\x12", 7}
	};
	ENV _environment = ENV::in;
	bool _envChanged = false;
	bool _running = false;

	jlib::Serial _serial;
	ros::NodeHandle _noder;
	ros::Subscriber _suber;
};

}