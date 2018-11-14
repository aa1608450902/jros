#include <serial_comlink/serial_comlink_node.h>
#include <error.h>
#include <fcntl.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <termios.h>
#include <string>
#include <exception>
#include <thread>
#include <deque>

namespace serial_comlink {

SerialComlinkNode::SerialComlinkNode() {

}

SerialComlinkNode::~SerialComlinkNode() {

}

void SerialComlinkNode::init() {
	_serial.set("/dev/ttyS0", 115200);
	std::vector<float> inExp{800, 1000, 1300}, outExp{100, 300, 600};
	_cameraAdjuster.setInExposure(inExp);
	_cameraAdjuster.setOutExposure(outExp);
}

void SerialComlinkNode::main() {
	init();

	_suber = _noder.subscribe("/position_info", 1, &SerialComlinkNode::positionCallback, this);
	std::thread td(&SerialComlinkNode::loop, this);
	ros::spin();
	td.join();
}

void SerialComlinkNode::loop() {
	std::string instruction;
	while (true) {
		if (!_running) break;
		instruction = _serial.readBuffer();
		ROS_INFO("Received a instruction [ %s ]", instruction.data());
		if (INS.find(instruction) == INS.end()) {
			continue;
		}
		int ins_num = INS.at(instruction);
		switch (ins_num) {
			case 1:
				ROS_INFO("System shutdown now...");
				system("halt");
				break;
			case 2:
				ros::param::set("start_image_process", "true");
				break;
			case 3:
				ros::param::set("start_image_process", "false");
				break;
			case 4:
				ros::param::set("auto_aiming", "true");
				break;
			case 5:
				ros::param::set("auto_aiming", "false");
				break;
			case 6:
				if (_environment == ENV::out)
					_envChanged = true;
				_environment = ENV::in;
				_cameraAdjuster.setInEnvironment();
				break;
			case 7:
				if (_environment == ENV::in)
					_envChanged = true;
				_environment = ENV::out;
				_cameraAdjuster.setOutEnvironment();
				break;
			default:
				;
		}
	}
}

void SerialComlinkNode::positionCallback(const laserd::Point32Plus::ConstPtr &msg) {
	std::string autoAim = "false";
	ros::param::param<std::string>("auto_aiming", autoAim, "false");

	std::string color = msg->color;
	std::string xxx, yyy, zzz;
	xxx = toHex(int(msg->x));
	yyy = toHex(int(msg->y));
	zzz = toHex(int(msg->z));

	/// 1. auto close
	/// 2. laser aim
	/// 3. laser instruction
	ROS_INFO("position info: ( %s, %s , %s , %s )", color.c_str(), xxx.c_str(), yyy.c_str(), zzz.c_str());
	bool laserOn = !(zzz == "0010");
	if (!laserOn) {

	}
	int64_t now = std::chrono::system_clock::now().time_since_epoch().count() / 1000000;

	/// adjust exposure
	std::string ins;
	if (_analyzer.running()) {
		if (now - _analyzer.beginTime > _analyzer.TWO_SECS) {
			ins = _analyzer.analyze();
			ROS_INFO("Analyzer analyze over ...");
			/// send parsed signal
			if (ins != "") {
				ROS_INFO("Serial port write action instruction [ %s ].", ins.c_str());
				_serial.write(ins);
			}
			_analyzer.clear();
		} else {
			_analyzer.append(laserOn, now);
		}
	} else if (laserOn) {
		ROS_INFO("analyzer start ...");
		_analyzer.start(true, now);
	}

	if (_cameraAdjuster.running()) {
		if (now - _cameraAdjuster.beginTime > _cameraAdjuster.HALF_SECS) {
			_cameraAdjuster.adjust();
			ROS_INFO("CameraAdjuster adjust over ...");
			_cameraAdjuster.clear();
		} else {
			_cameraAdjuster.append(laserOn, now);
		}
	} else if (laserOn) {
		ROS_INFO("cameraAdjuster start ...");
		_cameraAdjuster.start(laserOn, now);
	}

	std::string postitionInstruction;
	if (color == "0") {
		postitionInstruction = "dd" + xxx + yyy + zzz;
	} else if (color == "1") {
		postitionInstruction = "cc" + xxx + yyy + zzz;
	}
	if (postitionInstruction.length() <= 14) {
		std::stringstream ss;
		ss << std::hex << std::setw(2) << std::setfill('0') << postitionInstruction.length();
		postitionInstruction += ss.str();
	}
	ROS_INFO("Serial port write laser position instruction [ %s ].", postitionInstruction.c_str());
	_serial.write(postitionInstruction);

}

std::string SerialComlinkNode::toHex(int n) {
	std::stringstream ss;
	if (n >= 0) {
		ss << std::hex << std::setw(4) << std::setfill('0') << n;
	} else {
		ss << std::hex << std::setw(4) << std::setfill('0') << ((n + (1 << 16)) % (1 << 16));
	}
	std::string __r(ss.str());
	std::transform(__r.begin(), __r.end(), __r.begin(), ::toupper);
	return __r;
}


}