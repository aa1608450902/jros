//
// Created by u01 on 10/30/18.
//

#pragma once

#include <ros/ros.h>
#include <laserd/laserd.h>
#include <laserd/Point32Plus.h>
#include <sensor_msgs/Image.h>

namespace laserd {

class LaserdNode {
public:

	LaserdNode(int argc, char* argv[]);

	void start();

	void imageProc(const sensor_msgs::ImageConstPtr& msg);

private:

	ros::NodeHandle 	_handler;
	
	ros::Publisher 		_publisher;
	
	ros::Subscriber 	_subscriber;
	
	jlib::laserd 		_laserD;
	
	string 				_autoAim = "false";
	
	bool 				switch_aiming = false;

};

}


