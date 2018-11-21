//
// Created by u01 on 11/21/18.
//

#pragma once

#include <opencv2/core/core.hpp>

namespace jlib {

class CalcGray {
public:
	cv::Mat bgr2gray(const cv::Mat& img);
	cv::Mat calcGrayHist(const cv::Mat& img);
private:

};

}


