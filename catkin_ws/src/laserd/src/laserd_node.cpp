#include <csignal>
#include <laserd/LaserdNode.h>
#include <stdio.h>

void signalHandler(int signal) {
	ROS_INFO("signal handler... %d", signal);
	exit(0);
}

int main(int argc, char* argv[]) {
	std::signal(SIGINT, signalHandler);
    ros::init(argc, argv, "Position_deviation");
    laserd::LaserdNode laserNode(argc, argv);
    laserNode.start();
    return 0;
}
