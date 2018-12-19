#pragma once

//#define DEBUG

#include <deque>
#include <chrono>
#include <ros/ros.h>

#ifdef DEBUG
#include <string>
#include <iostream>
#include <vector>

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
}
}
#endif

typedef unsigned char uint8_t;
typedef unsigned long int uint64_t;

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

struct PulseAtom {
	uint8_t pre;
	uint8_t pos;
	uint8_t res;
	uint8_t eva;
	inline PulseAtom();
	inline void clear();
};
inline PulseAtom::PulseAtom()
: pre(0), pos(0), res(0), eva(0) {}
inline void PulseAtom::clear()
{ pre = 0; pos = 0; res = 0; eva = 0; }

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
	static int evaluate(std::vector<PulseAtom>& seq);
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

// evaluate level: 0 ~ 9
int Analyzer::evaluate(std::vector<PulseAtom>& seq) {
	for (auto& atom : seq) {
		float precent = float(atom.pre) / (atom.pre + atom.pos);
		if (atom.pre >= atom.pos) {
			atom.res = 1; atom.eva = 9;
		} else if (precent >= 0.4) {
			atom.res = 1; atom.eva = 7;
		} else {
			int level = 10 - int(precent * 10);
			atom.res = 0; atom.eva = static_cast<uint8_t>(level == 10 ? 9 : level);
		}
	}
	for (decltype(seq.size()) i = 1; i < 4; i++) {
		if (seq[i].res == seq[i + 3].res) {
			return -1;
//			if (seq[i].eva > seq[i + 3].eva)
//				seq[i + 3].res = static_cast<uint8_t>(1 - seq[i].res);
//			else
//				seq[i].res = static_cast<uint8_t>(1 - seq[i + 3].res);
		}
	}
	return 0;
}

int Analyzer::analyze() const {
	std::vector<Pulse> nseq = correct(_pulseSeq);
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
	if (nseq.size() < 20) {
		ROS_WARN("Only %d pulses, it's impossible to parse instruction.", int(nseq.size()));
		return -1;
	} else if (nseq.size() < 30) {
		ROS_WARN("Too little pulses! only %d.", int(nseq.size()));
	}
	std::vector<PulseAtom> comp;
	{
		int upWave = 0, downWave = 0;
		PulseAtom pulseAtom; pulseAtom.pre++;
		for (decltype(nseq.size()) i = 1; i < nseq.size(); i++) {
			if (nseq[i - 1].sig == 1 && nseq[i].sig == 0) {
				downWave++;
			} else if (nseq[i - 1].sig == 0 && nseq[i].sig == 1) {
				comp.emplace_back(pulseAtom);
				pulseAtom.clear();
				upWave++;
			}
			if (nseq[i].sig == 1)
				pulseAtom.pre++;
			else if (nseq[i].sig == 0)
				pulseAtom.pos++;
			if (i == nseq.size() - 1)
				comp.emplace_back(pulseAtom);
		}
		if (upWave != 6 || downWave != 7) {
			ROS_WARN("Pulse sequence isn't standard!");
			return -1;
		}
	}

	if (evaluate(comp) == -1)
		return -1;
#ifdef DEBUG
	for (auto& i : comp) {
		std::cout << "\t section 1. " << (int)i.pre
				  << "\t section 2. " << (int)i.pos
				  << "\t parse result 3. " << (int)i.res
				  << "\t eva. " << (int)i.eva
				  << std::endl;
	}
#endif
	return (comp[1].res << 2) | (comp[2].res << 1) | (comp[3].res);
}