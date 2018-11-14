//
// Created by u01 on 11/14/18.
//

#ifndef SERIAL_COMLINK_ANALYZER_H
#define SERIAL_COMLINK_ANALYZER_H

#include <deque>
#include <chrono>
#include <string>

#include <stdint.h>

namespace serial_comlink {

struct Impulse {
	/// true: 1; false: 0
	bool bit;
	/// unit: ms
	int64_t now;
};

class Analyzer {
private:
	bool _running = false;
	std::deque<Impulse> _impulses;

public:
	int64_t beginTime;
	const int64_t TWO_SECS = 2000;
	const std::string ERROR_CODE = "DDBBAA999966660E";

	bool running() { return _running; }
	void start(bool bit, int64_t now);
	void append(bool bit, int64_t now);
	std::string analyze();
	void clear();
};

}

#endif //SERIAL_COMLINK_ANALYZER_H
