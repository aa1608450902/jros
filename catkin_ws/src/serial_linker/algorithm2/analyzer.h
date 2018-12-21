#pragma once

#define DEBUG

#include <list>
#include <deque>
#include <vector>
#include <chrono>
#include <ros/ros.h>

#ifdef DEBUG
#include <string>
#include <iostream>
#include <vector>
#endif

typedef unsigned char       uint8_t;
typedef unsigned short int	uint16_t;
typedef unsigned int		uint32_t;
typedef unsigned long int   uint64_t;

enum AnalyzerSignal {
	off = 0, on = 1
};

struct Pulse {
	uint8_t sig;
	uint64_t time;
	inline Pulse(uint8_t s, uint64_t t);
};
inline Pulse::Pulse(uint8_t s, uint64_t t)
: sig(s), time(t) {}

class Analyzer {
public:
	inline Analyzer();
	~Analyzer() = default;
	inline void reset();
	inline bool isRunning() const;
	inline uint64_t getStartTime() const;
	static inline uint64_t getCurrentTime();
	void addAnalyzerSignal(
			AnalyzerSignal sig, uint64_t t);
	int analyze() const;
private:
	static std::vector<Pulse> correct(
			const std::deque<Pulse>& seq);
	static std::pair<int, int> statistics(
			const std::deque<Pulse>& seq);
	static int assess(const std::deque<Pulse>& seq);
	static int judge(const std::vector<Pulse>& seq);
private:
	bool _isRunning;
	uint64_t _startTime;
	std::deque<Pulse> _pulseSeq;
};

inline Analyzer::Analyzer()
: _isRunning(false), _startTime(0) {}

inline void Analyzer::reset()
{_isRunning = false; _startTime = 0; _pulseSeq.clear();}

inline bool Analyzer::isRunning() const
{return _isRunning;}

inline uint64_t Analyzer::getStartTime() const
{return _startTime;}

inline uint64_t Analyzer::getCurrentTime()
{return static_cast<uint64_t>(
std::chrono::system_clock::now().time_since_epoch().count() / 1000000);}

void Analyzer::addAnalyzerSignal(AnalyzerSignal sig, uint64_t t) {
	if (_pulseSeq.empty()) {
		_isRunning = true;
		_startTime = t;
	}
	_pulseSeq.emplace_back(Pulse(sig, t));
}

#ifdef DEBUG
namespace {
void print(const std::deque<Pulse>& seq) {
	std::string out, outTime;
	for (auto& it : seq) {
		out += std::to_string(it.sig) + " ";
		outTime += std::to_string(it.time) + " ";
	}
	std::cout << "pulse seq:\n\t" << out << std::endl;
	std::cout << "pulse time seq:\n\t" << outTime << std::endl;
}

void printv(const std::vector<Pulse>& seq) {
	std::string out, outTime;
	for (auto& it : seq) {
		out += std::to_string(it.sig) + " ";
		outTime += std::to_string(it.time) + " ";
	}
	std::cout << "correct pulse seq:\n\t" << out << std::endl;
	std::cout << "correct pulse time seq:\n\t" << outTime << std::endl;
	std::string span;
	for (decltype(seq.size()) i = 1; i < seq.size(); i++) {
		span += std::to_string(seq[i].time - seq[i - 1].time) + " ";
	}
	std::cout << "correct pulse time span seq:\n\t" << span << std::endl;
}
}
#endif

std::pair<int, int> Analyzer::statistics(const std::deque<Pulse>& seq) {
	std::pair<int, int> pr(0, 0);
	for (const auto& item : seq) {
		if (item.sig) {
			pr.first++;
		} else {
			pr.second++;
		}
	}
	return pr;
}

int Analyzer::assess(const std::deque<Pulse>& seq) {
	std::deque<Pulse> nseq = seq;
	int zero = 0, one = 0;
	for (auto i : nseq) {
		if (i.sig) one++;
		else zero++;
	}
//	if (zero == nseq.size() || one == nseq.size()) return -1;
	if (nseq.back().sig) {
		nseq.pop_back();
		one--;
	}
	float precent = static_cast<float>(one) / (zero + one);
	if (precent > 0.41) {
		ROS_INFO("zero: %d one: %d precent: %f return 1", zero, one, precent);
		return 1;
	} else {
		ROS_INFO("zero: %d one: %d precent: %f return 0", zero, one, precent);
		return 0;
	}
}

int Analyzer::judge(const std::vector<Pulse>& seq) {
	static int effect = 0;
	effect = 0;
	int downWave = 0, upWave = 0;
	std::deque<std::deque<Pulse>> seven = {{}, {}, {}, {}, {}, {}, {}};
	for (decltype(seq.size()) i = 0; i < seq.size(); i++) {
		// collect wave
		if (i != 0 && seq[i].sig == 0 && seq[i - 1].sig == 1) {
			downWave++;
		} else if (i != 0 && seq[i].sig == 1 && seq[i - 1].sig == 0) {
			upWave++;
		}
		// collect section
		if (seq[i].time < 200) {;
			seven[0].emplace_back(seq[i]);
		} else if (seq[i].time >= 200 && seq[i].time < 500) {
			seven[1].emplace_back(seq[i]);
		} else if (seq[i].time >= 500 && seq[i].time < 800) {
			seven[2].emplace_back(seq[i]);
		} else if (seq[i].time >= 800 && seq[i].time < 1100) {
			seven[3].emplace_back(seq[i]);
		} else if (seq[i].time >= 1100 && seq[i].time < 1400) {
			seven[4].emplace_back(seq[i]);
		} else if (seq[i].time >= 1400 && seq[i].time < 1700) {
			seven[5].emplace_back(seq[i]);
		} else if (seq[i].time >= 1700 && seq[i].time < 2000) {
			seven[6].emplace_back(seq[i]);
		}
	}
	if (downWave < 6 || downWave > 8) {
		ROS_WARN("Down wave don't meet requirement.");
		return -1;
	}
#ifdef DEBUG
	for (decltype(seven.size()) i = 0; i < seven.size(); i++) {
	std::cout << "[ " << i << " ] ";
	for (auto &j : seven[i]) {
		std::cout << (int)j.sig << " ";
	}
	std::cout << std::endl;
}
#endif
	for (decltype(seven.size()) i = 0; i < seven.size(); i++) {
		if (seven[i].back().sig == 1) {
			effect++;
		}
	}
	ROS_INFO("-> effect: %d", effect);
	if (effect > 2) {
		for (decltype(seven.size()) i = 0; i < 6; i++) {
			Pulse tmp = seven[i].back();
			seven[i].pop_back();
			seven[i + 1].emplace_front(tmp);
		}
	}
#ifdef DEBUG
	for (decltype(seven.size()) i = 0; i < seven.size(); i++) {
	std::cout << "[ " << i << " ] ";
	for (auto &j : seven[i]) {
		std::cout << (int)j.sig << " ";
	}
	std::cout << std::endl;
}
#endif

	int ins[] = {0, 0, 0, 0, 0, 0};
	for (decltype(seven.size()) i = 0; i < seven.size(); i++) {
		if (i == 0) {
			std::pair<int, int> rate = statistics(seven[0]);
			float precent = static_cast<float>(rate.first) / (rate.first + rate.second);
			if (precent < 0.2 || precent > 0.8) {
				ROS_WARN("Begin pulse don't meet requirement.");
				return -1;}
		} else {
			int ret = assess(seven[i]);
			if (ret == -1)
				return -1;
			ins[i - 1] = ret;
		}
	}
	ROS_INFO("-> ins: %d%d%d%d%d%d", ins[0], ins[1], ins[2], ins[3], ins[4], ins[5]);
	for (int i = 0; i < 3; i++) {
		if (ins[i] == ins[i + 3]) {
			ROS_WARN("Parsed instruction conflict.");
			return -1;
		}
	}
	effect = 0;
	return static_cast<uint8_t>(ins[0]) << 2 | static_cast<uint8_t>(ins[1]) << 1 | static_cast<uint8_t>(ins[2]);
}

std::vector<Pulse> Analyzer::correct(const std::deque<Pulse>& seq) {
	std::vector<Pulse> nseq;
	nseq.emplace_back(seq.front().sig, seq.front().time);
	for (decltype(seq.size()) i = 1; i < seq.size() - 1; i++) {
		const Pulse& pre = seq[i - 1];
		const Pulse& pos = seq[i + 1];
		Pulse curr = seq[i];
		if (curr.sig == 0 && pre.sig == 1 && pos.sig == 1
			&& (pos.time - pre.time <= 100)) {
			curr.sig = AnalyzerSignal::on;
			nseq.emplace_back(curr);
		} else {
			nseq.emplace_back(curr);
		}
	}
	nseq.emplace_back(Pulse(seq.back().sig, seq.back().time));
	return nseq;
}

int Analyzer::analyze() const {
	std::vector<Pulse> nseq = correct(_pulseSeq);
	const uint64_t bg = nseq.front().time;
	for (auto& item : nseq) {
		item.time = item.time - bg;
	}
	std::string sequence;
	for (auto& i : nseq) {
		sequence += std::to_string(i.sig) + " ";
	}
	ROS_INFO("### impulse %s", sequence.data());
#ifdef DEBUG
	print(_pulseSeq);
	printv(nseq);
#endif
	// First of all, the pulse must be
	if (nseq.size() < 18) {
		ROS_WARN("Only %d pulses, it's impossible to parse instruction.", int(nseq.size()));
		return -1;
	} else if (nseq.size() < 30) {
		ROS_WARN("Too little pulses! only %d.", int(nseq.size()));
	}

	return judge(nseq);
}