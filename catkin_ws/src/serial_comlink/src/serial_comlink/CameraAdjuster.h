//
// Created by u01 on 11/14/18.
//

#ifndef SERIAL_COMLINK_CAMERAADJUSTER_H
#define SERIAL_COMLINK_CAMERAADJUSTER_H

#include <deque>
#include <chrono>
#include <string>
#include <vector>

#include <Analyzer.h>

namespace serial_comlink {

class CameraAdjuster {
private:
	bool _running = false;
	std::deque<Impulse> _impulses;
	bool _environmentIn = true;
	uint8_t _inExpIndex = 0;
	uint8_t _outExpIndex = 0;
	std::vector<float> _exposureIn;
	std::vector<float> _exposureOut;
	void setExposure(float exp);
	void setExposure();
public:
	void setInEnvironment() {_environmentIn = true;}
	void setOutEnvironment() {_environmentIn = false;}
	void setInExposure(const std::vector<float>& exp) {_exposureIn = exp;}
	void setOutExposure(const std::vector<float>& exp) {_exposureOut = exp;}
	int64_t beginTime;
	const int64_t HALF_SECS = 500;
	const int64_t ONE_SECS = 1000;
	bool running() { return _running; }
	void start(bool bit, int64_t now);
	void append(bool bit, int64_t now);
	void clear();
	void adjust();
};

}

#endif //SERIAL_COMLINK_CAMERAADJUSTER_H
