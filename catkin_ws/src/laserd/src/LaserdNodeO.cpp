//
// Created by u01 on 10/30/18.
//

#include <laserd/LaserdNode.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <string>

namespace laserd {

LaserdNode::LaserdNode(int argc, char* argv[]) {
	_publisher = _handler.advertise<laserd::Point32Plus>("position_info", 1);
	_subscriber = _handler.subscribe("/pylon_camera_node/image_raw", 1, &LaserdNode::imageProc, this);
	ROS_INFO("init [ Position_deviation ], publish [ position_info ] topic, subscribe [ /pylon_camera_node/image_raw ] topic.");
}

void LaserdNode::start() {
	ROS_INFO("laserd node start working...");
//	vector<int> red_hsv_1_low{1,2,3}, red_hsv_2_low;
//	string s1{"hehe"}, s2;
//	ros::param::get("red_hsv_2_low", red_hsv_2_low);
//	red_hsv_2_low = ros::param::param<vector<int>>("red_hsv_2_low", red_hsv_1_low);
//	s2 = ros::param::param<string>("red_hsv_2_low", s1);
//	std::string output;
//	for (auto& i : red_hsv_2_low)
//		output += std::to_string(i) + ",";
//	ROS_INFO("red_hsv_1_low: %s", output.data());
//	ROS_INFO("s2: %s", s2.data());

	ros::spin();
}

void LaserdNode::imageProc(const sensor_msgs::ImageConstPtr& msg) {
	string signal_bool = "false";
	if (ros::param::has("start_image_process"))
		ros::param::get("start_image_process", signal_bool);
	if (ros::param::has("auto_aiming"))
		ros::param::get("auto_aiming", _autoAim);

	if (_autoAim == "true") {
		if (switch_aiming) {
			switch_aiming = false;
			return;
		} else {
			switch_aiming = true;
		}
	}

	cv_bridge::CvImagePtr image_ptr;
	try {
		image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::Point g_center;
	laserd::Point32Plus g_center_msg;
	if (!_laserD.searchGreenLaser(image_ptr->image, g_center)) {
		g_center_msg.color = "0";
		g_center_msg.x = g_center.x;
		g_center_msg.y = g_center.y;
		g_center_msg.z = 0;
	} else {
		g_center_msg.color = "0";
		g_center_msg.x = 0;
		g_center_msg.y = 0;
		g_center_msg.z = 16;
	}
	// ROS_INFO("green laser ( %d , %d )", g_center.x, g_center.y);
	_publisher.publish(g_center_msg);

	cv::Point r_center;
	if (_autoAim == "true") {
		laserd::Point32Plus r_center_msg;
		if (!_laserD.searchRedLaser(image_ptr->image, r_center)) {
			r_center_msg.color = "1";
			r_center_msg.x = r_center.x;
			r_center_msg.y = r_center.y;
			r_center_msg.z = 0;
		} else {
			r_center_msg.color = "1";
			r_center_msg.x = 0;
			r_center_msg.y = 0;
			r_center_msg.z = 16;
		}
		// ROS_INFO("red laser ( %d , %d )", r_center.x, r_center.y);
		_publisher.publish(r_center_msg);
	}
	ROS_INFO("green laser ( %d, %d )\t\tred laser ( %d, %d )", g_center.x, g_center.y, r_center.x, r_center.y);
}

}

