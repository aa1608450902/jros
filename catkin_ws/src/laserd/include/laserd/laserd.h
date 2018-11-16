//
// Created by u01 on 10/30/18.
//

#pragma once

/// stdc++
#include <set>
#include <string>
using namespace std;
/// opencv2
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

namespace jlib {

	class laserd {

	public:

		static int searchRedLaser(const cv::Mat& image, cv::Point& __r);

		static int searchGreenLaser(const cv::Mat& image, cv::Point& __r);

		static int searchGreenLaserO(const cv::Mat& image, cv::Point& __r);

		static cv::Mat filter(const cv::Mat& src);

		static cv::Mat seperate(const cv::Mat& src, double factor = 0.8);

		static cv::Mat calcGrayHist(const cv::Mat& img);

		static int calcCenterLaser(const cv::Mat& binary_image, cv::Point& center);

		static int calcCenterLaserO(const cv::Mat& binary_image, cv::Point& center);

		static int searchPeakDomain(const cv::Mat& src);

		static void expandDomain(const cv::Mat& map, std::pair<int, int> start, std::set<std::pair<int, int>>& collection);

	private:

		static bool hasPointInSet(const std::set<std::pair<int, int>>& collection, const std::pair<int, int>& point);

	};

}
