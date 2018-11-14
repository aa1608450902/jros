//
// Created by u01 on 11/14/18.
//

#include "Analyzer.h"

#include <vector>
#include <ros/ros.h>

namespace serial_comlink {

void Analyzer::start(bool bit, int64_t now) {
	if (!_impulses.empty()) {
		_impulses.clear();
	}
	_running = true;
	beginTime = now;
	Impulse impulse{bit, now};
	_impulses.push_back(impulse);
}
void Analyzer::append(bool bit, int64_t now) {
	Impulse impulse{bit, now};
	_impulses.push_back(impulse);
}

void Analyzer::clear() {
	_running = false;
	beginTime = 0;
	_impulses.clear();
}

std::string Analyzer::analyze() {
	if (_impulses.size() < 20) {
		ROS_WARN("Analyzer just collected [ %d ] images / [ %d ] ms. [ 20 ] is best.", _impulses.size(), TWO_SECS);
	}
	std::vector<bool> __r;
	int count = 0;

	std::stringstream ss, st;
	for (auto& per : _impulses) {
		ss << (per.bit ? "1" : "0") << " ";
		st << std::to_string(per.now) << " ";
	}
	ROS_INFO("impulse: %s", ss.str().data());
	// ROS_INFO("time: %s", st.str().data());

	bool intervalControl = false;
	int64_t intervalBegin;
	size_t intervalIndex = 0, intervalBeginIndex = 0, intervalEndIndex = 0;
	// ROS_INFO(" ----- 1 -----");
	auto pre = _impulses.begin(), p = _impulses.begin() + 1;
	while (p != _impulses.end()) {
		// ROS_INFO(" ----- 1 ----- 1 ------ ");
		if (pre->bit && !p->bit) {
			intervalControl = true;
			intervalBegin = pre->now;
			intervalBeginIndex = intervalIndex;
		}
		if (!pre->bit && p->bit) {
			if (intervalControl) {
				if (p->now - intervalBegin < 100) {
					intervalEndIndex = intervalIndex;
					for (size_t i = intervalBeginIndex; i < intervalEndIndex; ++i) {
						if (!_impulses[i].bit) {
							_impulses[i].bit = true;
						}
					}
				}
			}
			intervalControl = false;
		}
		intervalIndex++;
		pre++;
		p++;
	}
	// ROS_INFO(" ----- 2 -----");
	pre = _impulses.begin(), p = _impulses.begin() + 1;
	while (p != _impulses.end()) {
		if (pre->bit && !p->bit) {
			count++;
		}
		pre++;
		p++;
	}
	if (count < 7) {
		ROS_WARN("Analyzer only found [ %d ] impulses.", count);
		return "";
	}
	// ROS_INFO(" ----- 3 -----");
	std::deque<Impulse>::iterator last, lastPre;
	std::vector<std::pair<int64_t, int64_t>> range;
	int pulseCount = 0;
	pre = _impulses.begin(), p = _impulses.begin() + 1;
	while (p != _impulses.end()) {
		int64_t min, max;
		if (pre->bit && !p->bit) {
			pulseCount++;
			if (pulseCount == 1) {
				min = pre->now - beginTime;
				max =   p->now - beginTime;
				range.emplace_back(min, max);
				last = p;
				lastPre = pre;
			} else if (pulseCount % 2 == 1) {
				min = pre->now - last->now;
				max = p->now - lastPre->now;
				range.emplace_back(min, max);
			}
		} else if (!pre->bit && p->bit) {
			pulseCount++;
			min = pre->now - last->now;
			max = p->now - lastPre->now;
			range.emplace_back(min, max);
			last = p;
			lastPre = pre;
		}
	}
	// ROS_INFO(" ----- 4 -----");
	std::vector<bool> result;
	size_t index = 0;
	while (index < range.size()) {
		if ((index == 0 || index == 1)) {
			if (range[index].second > 100) {
				return "";
			}
		} else {
			if (!(index % 2)) {
				if (range[index].first > 100) {
					__r.push_back(true);
				} else {
					if (range[index].second == 100) {
						__r.push_back(false);
					} else if (range[index].second > 100) {
						/// analyze another side
						if (range[index + 1].first > 100) {
							__r.push_back(false);
						} else {
							if (range[index + 1].second == 100) {
								__r.push_back(true);
							} else if (range[index + 1].second > 100) {
								/// unknown statement
								ROS_WARN("Analyzer enter ambiguous branch, the impulse 1st_max > 100 ms and 2nd_max > 100 ms.");
							} else { /// max < 100 (X)
								/// impossible branch
								ROS_WARN("Analyzer enter impossible branch which impulse 2nd_max < 100 ms.");
							}
						}
					} else { /// max < 100 (X)
						/// impossible branch
						ROS_WARN("Analyzer enter impossible branch which impulse 1st_max < 100 ms.");
					}
				}
			}
		}
		index++;
	}
	// ROS_INFO(" ----- 5 -----");
	if (__r.size() < 6) {
		ROS_WARN("Analyzer parse problem, too few impulses: [ %d ]", __r.size());
		return ERROR_CODE;
	}
	int correctNumber = 0;
	for (auto i = 0; i < 3; ++i) {
		if ((__r[i] && !__r[i + 3]) || (!__r[i] && __r[i + 3])) {
			correctNumber++;
		}
	}
	std::string ins;
	for (auto i = 0; i < 6; ++i) {
		if (__r[i]) ins += "1"; else ins += "0";
	}

	if (correctNumber == 3) {
		std::string preThree = std::to_string(std::stoi(ins.substr(0, 3), nullptr, 2));
		std::string postThree = std::to_string(7 - std::stoi(ins.substr(0, 3), nullptr, 2));
		std::string msg = "ddbbaa0" + preThree + "0" + preThree + "0" + postThree + "0" + postThree + "0e";
		return msg;
	} else {
		ROS_WARN("Analyzer parse problem, instruction: [ %s ]", ins.c_str());
		return ERROR_CODE;
	}
}

};