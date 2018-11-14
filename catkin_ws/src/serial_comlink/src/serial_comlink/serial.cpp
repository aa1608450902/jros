#include "serial.h"

#include <error.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <string>
#include <thread>
#include <exception>

namespace jlib {

Serial::Serial()
	:_ttyS0_fd(-1) {

}

Serial::~Serial() {
	close(_ttyS0_fd);
}

// dev: "/dev/ttyUSB0"
void Serial::set(const std::string& dev, int speed, int bits, char validate, int stop) {
	_ttyS0_fd = ::open(dev.data(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (_ttyS0_fd == -1) {
		throw std::runtime_error("Cannot open " + dev + " device.");
		return;
	}
	struct termios oldttys1, newttys1;
	if (tcgetattr(_ttyS0_fd, &oldttys1) != 0) {
//		ROS_INFO("Store previous serial setting failed.");
	}
	bzero(&newttys1, sizeof(newttys1));
	newttys1.c_cflag |= (CLOCAL|CREAD);
	newttys1.c_cflag &= ~CSIZE;
	switch (bits) {
		case 7:
			newttys1.c_cflag |= CS7;
			break;
		case 8:
			newttys1.c_cflag |= CS8;
			break;
		default:
			newttys1.c_cflag = oldttys1.c_cflag;
//			throw std::runtime_error("Serial data bit error [ " + std::to_string(bits) + " ] bits.");
	}
	switch (validate) {
		case '0':  /// odd
			newttys1.c_cflag |= PARENB;
			newttys1.c_iflag |= (INPCK | ISTRIP);
			newttys1.c_cflag |= PARODD;
			break;
		case 'E':  /// even
			newttys1.c_cflag |= PARENB;
			newttys1.c_iflag |= (INPCK | ISTRIP);
			newttys1.c_cflag &= ~PARODD;
			break;
		case 'N':  /// without check bit
			newttys1.c_cflag &= ~PARENB;
			break;
		default:
			newttys1.c_cflag = oldttys1.c_cflag;
//			throw std::runtime_error(std::string("Serial check bit error [ ") + validate + " ]");
	}
	switch (speed) {
		case 2400:
			cfsetispeed(&newttys1, B2400);
			cfsetospeed(&newttys1, B2400);
			break;
		case 4800:
			cfsetispeed(&newttys1, B4800);
			cfsetospeed(&newttys1, B4800);
			break;
		case 9600:
			cfsetispeed(&newttys1, B9600);
			cfsetospeed(&newttys1, B9600);
			break;
		case 115200:
			cfsetispeed(&newttys1, B115200);
			cfsetospeed(&newttys1, B115200);
			break;
		default:
			cfsetispeed(&newttys1, B9600);
			cfsetospeed(&newttys1, B9600);
	}
	if (stop == 1) {
		newttys1.c_cflag &= ~CSTOPB;
	} else if (stop == 2) {
		newttys1.c_cflag |= CSTOPB;
	}
	if ((tcsetattr(_ttyS0_fd, TCSANOW, &newttys1))!=0) {
		throw std::runtime_error("Activate serial setting failed.");
	}
}

std::string Serial::readBuffer() {
	const int BUF_SIZE = 100;
	char buffer[BUF_SIZE];
	if (::read(_ttyS0_fd, buffer, BUF_SIZE) == -1) {
		throw std::runtime_error("Read serial failed");
	}
	std::string buf(buffer);
	return buf;
}

void Serial::write(const std::string& data) {
	if (::write(_ttyS0_fd, data.c_str(), data.length()) != data.length()) {
		throw std::runtime_error("Write serial failed");
	}
}



}