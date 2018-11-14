#pragma once

#include <string>

namespace jlib {

class Serial {
public:
	Serial();
	virtual ~Serial();

	void set(const std::string& dev, int speed, int bits = 8, char validate = 'N', int stop = 1);
	std::string readBuffer();
	void write(const std::string& data);
private:
	int _ttyS0_fd;
};

}