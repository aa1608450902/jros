//
// Created by u01 on 11/21/18.
//

#include "CalcGray.h"

#include <opencv2/imgproc/imgproc.hpp>

namespace jlib {

cv::Mat CalcGray::bgr2gray(const cv::Mat &img) {
	cv::Mat dst;
	cv::cvtColor(img, dst, CV_BGR2GRAY);
	return dst;
}

cv::Mat CalcGray::calcGrayHist(const cv::Mat &img) {
	cv::Mat histogram = cv::Mat::zeros(cv::Size(256, 1), CV_32SC1);
	int rows = img.rows, cols = img.cols;
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			int index = int(img.at<uchar>(i, j));
			histogram.at<int>(0, index) += 1;
		}
	}
	return histogram;
}

}

