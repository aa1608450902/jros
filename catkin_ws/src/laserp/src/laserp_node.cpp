#include <ros/ros.h>
/// self define msg
#include <laserp/Point32Plus.h>
/// ros cvbridge
#include <cv_bridge/cv_bridge.h>
/// ros stdmsg
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
/// opencv2
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
/// stdc++
#include <vector>

ros::Publisher publisher;

void imageProcess(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr image_ptr;
	try {
		image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	} catch (...) {
		ROS_ERROR("cv_bridge exception: unknown error.");
		return;
	}

	cv::Mat hsvImage;
	cv::cvtColor(image_ptr->image, hsvImage, CV_BGR2HSV);

	/// green laser position
	{
		std::vector<int> greenHSVLowerLimit{ 35,  43, 200};
		std::vector<int> greenHSVUpperLimit{ 77, 255, 255};
		cv::Mat mask;
		cv::inRange(hsvImage,
				cv::Scalar(greenHSVLowerLimit[0], greenHSVLowerLimit[1], greenHSVLowerLimit[2]),
				cv::Scalar(greenHSVUpperLimit[0], greenHSVUpperLimit[1], greenHSVUpperLimit[2]),
				mask);

		/// calculate laser's center
		int xSum = 0, xCount = 0;
		int ySum = 0, yCount = 0;
		for (int r = 0; r < mask.rows; ++r) {
			for (int c = 0; c < mask.cols; ++c) {
				if (int(mask.at<uchar>(r, c) > 0)) {
					xCount++; xSum += c;
					yCount++; ySum += r;
				}
			}
		}
		laserp::Point32Plus centerMsg;
		if (!xCount || !yCount) {
			centerMsg.color = "0";
			centerMsg.x = 0;
			centerMsg.y = 0;
			centerMsg.z = 16;
		} else {
			centerMsg.color = "0";
			centerMsg.x = int(xSum / xCount - mask.cols / 2);
			centerMsg.y = int(mask.rows / 2 - ySum / yCount);
			centerMsg.z = 0;
		}
		publisher.publish(centerMsg);
	}

	/// red laser position
	{
		std::vector<int> redHSVPart1LowerLimit{  0,  43, 200};
		std::vector<int> redHSVPart1UpperLimit{ 10, 255, 255};
		std::vector<int> redHSVPart2LowerLimit{156,  43, 200};
		std::vector<int> redHSVPart2UpperLimit{180, 255, 255};
		cv::Mat part1, part2, mask;
		cv::inRange(hsvImage,
		            cv::Scalar(redHSVPart1LowerLimit[0], redHSVPart1LowerLimit[1], redHSVPart1LowerLimit[2]),
		            cv::Scalar(redHSVPart1UpperLimit[0], redHSVPart1UpperLimit[1], redHSVPart1UpperLimit[2]),
		            part1);
		cv::inRange(hsvImage,
		            cv::Scalar(redHSVPart2LowerLimit[0], redHSVPart2LowerLimit[1], redHSVPart2LowerLimit[2]),
		            cv::Scalar(redHSVPart2UpperLimit[0], redHSVPart2UpperLimit[1], redHSVPart2UpperLimit[2]),
		            part2);
		cv::bitwise_or(part1, part2, mask);

		/// calculate laser's center
		int xSum = 0, xCount = 0;
		int ySum = 0, yCount = 0;
		for (int r = 0; r < mask.rows; ++r) {
			for (int c = 0; c < mask.cols; ++c) {
				if (int(mask.at<uchar>(r, c) > 0)) {
					xCount++; xSum += c;
					yCount++; ySum += r;
				}
			}
		}
		laserp::Point32Plus centerMsg;
		if (!xCount || !yCount) {
			centerMsg.color = "1";
			centerMsg.x = 0;
			centerMsg.y = 0;
			centerMsg.z = 16;
		} else {
			centerMsg.color = "1";
			centerMsg.x = int(xSum / xCount - mask.cols / 2);
			centerMsg.y = int(mask.rows / 2 - ySum / yCount);
			centerMsg.z = 0;
		}
		publisher.publish(centerMsg);
	}

}

int main(int argc, char* argv[]) {
	ROS_INFO("laserp node starting ...");
	ros::init(argc, argv, "Position_deviation");

	ros::NodeHandle handler;
	ros::Subscriber subscriber = handler.subscribe("/pylon_camera_node/image_raw", 1, imageProcess);

	publisher = handler.advertise<laserp::Point32Plus>("position_info", 1);

	ROS_INFO("Enter main loop - ros::spin().");
	ros::spin();

    return 0;
}
