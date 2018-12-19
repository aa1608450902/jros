#ifndef __SERIALPORT_H__
#define __SERIALPORT_H__

#include <string>
extern "C" {
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
};

class SerialPort {
public:
	inline SerialPort(
		const std::string& dev, int baudRate,
		int cs = 8, char validation = 'N', int stopBit = 1);
	~SerialPort();
	int connect();
	std::string readBuffer() const;
	void writeBuffer(const std::string& data);
private:
	int _fd;
	const std::string _dev;
	int _baudRate;
	int _cs;
	char _validation;
	int _stopBit;
};

inline SerialPort::SerialPort(const std::string &dev, int baudRate, int cs, char validation, int stopBit)
: _fd(0), _dev(dev), _baudRate(baudRate), _cs(cs), _validation(validation), _stopBit(stopBit) {}

SerialPort::~SerialPort() {
	close(_fd);
}

int SerialPort::connect() {
	_fd = open(_dev.data(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (_fd == -1) return -1;
	struct termios oldttys1, newttys1;
	if (tcgetattr(_fd, &oldttys1) != 0) ;
	bzero(&newttys1, sizeof(newttys1));
	newttys1.c_cflag |= (CLOCAL|CREAD);
	newttys1.c_cflag &= ~CSIZE;
	switch (_cs) {
		case 7:
			newttys1.c_cflag |= CS7;
			break;
		case 8:
			newttys1.c_cflag |= CS8;
			break;
		default:
			newttys1.c_cflag = oldttys1.c_cflag;
	}
	switch (_validation) {
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
	}
	switch (_baudRate) {
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
	if (_stopBit == 1)
		newttys1.c_cflag &= ~CSTOPB;
	else if (_stopBit == 2)
		newttys1.c_cflag |= CSTOPB;
	if ((tcsetattr(_fd, TCSANOW, &newttys1))!=0)
		return -1;
	return 0;
}

std::string SerialPort::readBuffer() const {
	const int BUF_SIZE = 100;
	char buffer[BUF_SIZE] = {0};
	if (read(_fd, buffer, BUF_SIZE) == -1) {
		return "";
	}
	std::string buf(buffer);
	return buf;
}

void SerialPort::writeBuffer(const std::string& data) {
	write(_fd, data.c_str(), data.length());
}



#endif //__SERIALPORT_H__
