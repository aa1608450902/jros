#include <string>
#include <thread>
#include <sstream>
#include <bitset>
extern "C" {
#include <signal.h>
};
// ros
#include <ros/ros.h>
#include <laserp/Point32Plus.h>
// .h
#include "adjuster.h"
#include "analyzer.h"
#include "SerialPort.h"

#define DarkExpo     800
#define LightExpo    800
#define AnalyzeTime  2000
#define ErrorIns     "DDBBAA999966660E"

bool gIsRunning;
Analyzer *gAnalyzer;
SerialPort *gSerialPort;

const std::map<int, std::string> gInsMap = {
	{0b000, "ddbbaa0000ffff0e"}, {0b001, "ddbbaa0101fefe0e"},
	{0b010, "ddbbaa0202fdfd0e"}, {0b011, "ddbbaa0303fcfc0e"},
	{0b100, "ddbbaa0404fbfb0e"}, {0b101, "ddbbaa0505fafa0e"},
	{0b110, "ddbbaa0606f9f90e"}, {0b111, "ddbbaa0707f8f80e"}
};
const std::map<std::string, int> gSlavaIns = {
	{"\xcc\x01\x01\x99", 1}, {"\xcc\x01\x01\x06", 2},
	{"\xcc\x01\x01\x00", 3}, {"\xcc\x01\x01\x05", 4},
	{"\xcc\x01\x01\x01", 5}, {"\xcc\x01\x01\x11", 6},
	{"\xcc\x01\x01\x12", 7}
};

static std::string toHex(int);
static void signalHandle(int);
static void positionCallback(
		const laserp::Point32Plus::ConstPtr &msg);
static void serialLoop();

int main(int argc, char* argv[]) {
	ROS_INFO("start serial linker node ...");

	// init variables
	signal(SIGINT, signalHandle);
	gIsRunning   = true;
	gAnalyzer    = new Analyzer();
	gSerialPort  = new SerialPort("/dev/ttyS0", 115200);
	if (gSerialPort->connect() == -1) {
		ROS_ERROR("Serial port init failed! exiting ...");
		return EXIT_FAILURE;
	}

	// init ros node
	ros::init(argc, argv, "SerialCom");
	ros::NodeHandle nodeHandle;
	ros::Subscriber subscriber = nodeHandle.subscribe("/position_info", 1, positionCallback);

	// create a thread receive serial info
	std::thread td(serialLoop);
	td.detach();

	// begin main thread
	ros::spin();

    return 0;
}

void positionCallback(const laserp::Point32Plus::ConstPtr &msg) {
	std::string color = msg->color;
	std::string xxx = toHex(int(msg->x)), yyy = toHex(int(msg->y)), zzz = toHex(int(msg->z));

	if (color == "0") {
		auto laserOn = !(xxx == "0000" && yyy == "0000") ? AnalyzerSignal::on : AnalyzerSignal::off;
		uint64_t now = Analyzer::getCurrentTime();
		if (gAnalyzer->isRunning()) {
			if (now - gAnalyzer->getStartTime() > AnalyzeTime) {
				int ins = gAnalyzer->analyze();
				std::string inss;
				if (ins == -1) inss = ErrorIns;
				if (gInsMap.find(ins) != gInsMap.end())
					inss = gInsMap.at(ins);
				else
					inss = ErrorIns;
				char bbuf[65]; void *p = bbuf;
				*(reinterpret_cast<unsigned char *>(p))     = 0xdd;
				*(reinterpret_cast<unsigned char *>(p) + 1) = 0xbb;
				*(reinterpret_cast<unsigned char *>(p) + 2) = 0xaa;
				*(reinterpret_cast<unsigned char *>(p) + 3) = static_cast<unsigned char>(ins);
				*(reinterpret_cast<unsigned char *>(p) + 4) = static_cast<unsigned char>(ins);
				*(reinterpret_cast<unsigned char *>(p) + 5) = static_cast<unsigned char>(255 - ins);
				*(reinterpret_cast<unsigned char *>(p) + 6) = static_cast<unsigned char>(255 - ins);
				*(reinterpret_cast<unsigned char *>(p) + 7) = 0x0e;
				bbuf[64] = '\0';
				std::string sss(bbuf);
				gSerialPort->writeBuffer(sss);
				ROS_INFO(">>>> analyze result: %s", inss.data());
				gAnalyzer->reset();
			} else {
				gAnalyzer->addAnalyzerSignal(laserOn, now);
			}
		} else if (laserOn) {
			ROS_INFO(">>>> analyzer start");
			gAnalyzer->addAnalyzerSignal(laserOn, now);
		}
	}

	std::string posIns;
	if (color == "0")
		posIns = "dd" + xxx + yyy + zzz;
	else if (color == "1")
		posIns = "cc" + xxx + yyy + zzz;
	if (posIns.length() <= 14) {
		std::stringstream ss;
		ss << std::hex << std::setw(2) << std::setfill('0') << posIns.length();
		posIns += ss.str();
	}
//	ROS_INFO("Serial port write laser position instruction [ %s ].", posIns.c_str());
	std::bitset<64> posInsBin(std::stoul(posIns, 0, 16));
	gSerialPort->writeBuffer(posInsBin.to_string());
}

void serialLoop() {
	std::string slaveIns;
	unsigned char env = 0x00; // represent inner environment
	while (gIsRunning) {
		slaveIns = gSerialPort->readBuffer();
		if (slaveIns.empty()) continue;
//		ROS_INFO("Received a instruction [ %s ]", slaveIns.data());
		if (gSlavaIns.find(slaveIns) == gSlavaIns.end()) {
			continue;
		}
		int ins_num = gSlavaIns.at(slaveIns);
		switch (ins_num) {
			case 1:
				ROS_INFO("System shutdown now...");
				system("echo gg | sudo -S init 0");
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
				if (env == 0x01) { // represent light
					Adjuster::setCameraExp(DarkExpo);
					ROS_INFO("Changed exposure: %d", DarkExpo);
					env = 0x00;
				}
				break;
			case 7:
				if (env == 0x00) { // represent dark
					Adjuster::setCameraExp(LightExpo);
					ROS_INFO("Changed exposure: %d", LightExpo);
					env = 0x01;
				}
				break;
			default:
				;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

void signalHandle(int sig) {
	switch (sig) {
		case SIGINT:
			ROS_INFO("SIGINT, Exiting ...");
			break;
		default: ;
	}
}

std::string toHex(int x) {
	std::stringstream ss;
	if (x >= 0) {
		ss << std::hex << std::setw(4) << std::setfill('0') << x;
	} else {
		ss << std::hex << std::setw(4) << std::setfill('0') << ((x + (1 << 16)) % (1 << 16));
	}
	std::string __r(ss.str());
	std::transform(__r.begin(), __r.end(), __r.begin(), ::toupper);
	return __r;
}
