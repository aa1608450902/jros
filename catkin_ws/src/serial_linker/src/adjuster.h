#ifndef __ADJUSTER_H__
#define __ADJUSTER_H__

#include <ros/ros.h>
#include <camera_control_msgs/SetExposure.h>

class Adjuster {
public:
	static void setCameraExp(float exp);
private:

};

void Adjuster::setCameraExp(float exp) {
	camera_control_msgs::SetExposure setExp;
	setExp.request.target_exposure = exp;
	ros::service::call("/pylon_camera_node/set_exposure", setExp);
}

#endif //__ADJUSTER_H__
