//
// Created by u01 on 11/14/18.
//

#include "CameraAdjuster.h"
#include <ros/ros.h>
#include <camera_control_msgs/SetExposure.h>

namespace serial_comlink {
	void CameraAdjuster::start(bool bit, int64_t now) {
		if (!_impulses.empty()) {
			_impulses.clear();
		}
		_running = true;
		beginTime = now;
		Impulse impulse{bit, now};
		_impulses.push_back(impulse);
	}
	void CameraAdjuster::append(bool bit, int64_t now) {
		Impulse impulse{bit, now};
		_impulses.push_back(impulse);
	}

	void CameraAdjuster::clear() {
		_running = false;
		beginTime = 0;
		_impulses.clear();
	}

	void CameraAdjuster::adjust() {
		if (_impulses.size() < 20) {
			ROS_WARN("CameraAdjuster just collected [ %d ] images / [ %d ] ms. [ 10 ] is best.", _impulses.size(), HALF_SECS);
		}
		std::stringstream ss, st;
		for (auto& per : _impulses) {
			ss << (per.bit ? "1" : "0") << " ";
			st << std::to_string(per.now) << " ";
		}
		ROS_INFO("camera impulse: %s", ss.str().data());
		// ROS_INFO("camera time: %s", st.str().data());

		int laserCount = 0;
		for (auto& per : _impulses) {
			if (per.bit) {
				laserCount++;
			}
		}
		if (laserCount < 2) {
			setExposure();
		}
		// ROS_INFO(" ++++++ 1 ++++++ ");
		if (laserCount == _impulses.size()) {
			return;
		}
		auto pre = _impulses.begin(), p = _impulses.begin() + 1;
		int count = 0;

		int downImpulse = 0;
//		bool maybeInstruction = false;
		// ROS_INFO(" ++++++ 2 ++++++ ");
		while (p != _impulses.end()) {
			if (p->bit) {
				laserCount++;
			}
			if (pre->bit && !p->bit) {
				downImpulse++;
			}
			if (downImpulse == 1) {
				if (pre->now - beginTime > 100) {
					setExposure();
					return;
				} else {
					break;
//					maybeInstruction = true;
				}
			}
			pre++;
			p++;
		}
		// ROS_INFO(" ++++++ 3 ++++++ ");
		if (laserCount - count < count) {
			setExposure();
			return;;
		}
	}

	void CameraAdjuster::setExposure(float exp) {
		camera_control_msgs::SetExposure setExposure1;
		setExposure1.request.target_exposure = exp;
		ros::service::call("/pylon_camera_node/set_exposure", setExposure1);
	}

	void CameraAdjuster::setExposure() {
		if (_environmentIn) {
			_inExpIndex++;
			setExposure(_exposureIn[_inExpIndex % _exposureIn.size()]);
			ROS_INFO("Changed inner exposure [ %f ]", _exposureIn[_inExpIndex % _exposureIn.size()]);
		} else {
			_outExpIndex++;
			setExposure(_exposureOut[_outExpIndex % _exposureOut.size()]);
			ROS_INFO("Changed external exposure [ %f ]", _exposureOut[_outExpIndex % _exposureOut.size()]);
		}
	}

}