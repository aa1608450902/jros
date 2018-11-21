#include <iostream>
#include <string>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <CalcGray.h>
#include <stdlog.h>

int main(int argc, char* argv[]) {
    if (argc != 2) {
        ERROR("usage [ " + string(argv[0]) + " + path ]");
        exit(-1);
    }

    string path(argv[1]);
    cv::Mat img;
    try {
        img = cv::imread(path);
    } catch (...) {
        ERROR("read image error");
        exit(-2);
    }
    if (img.empty()) {
        ERROR("read image error");
        exit(-2);
    }

    jlib::CalcGray calcGray;
    cv::Mat gray = calcGray.bgr2gray(img);
    cv::Mat hist = calcGray.calcGrayHist(gray);

    int r_pixel = 0, c_pixel = 0;
    r_pixel = gray.rows;
    c_pixel = gray.cols;

    int min = 0, max = 0;
    for (int i = 0; i < 256; ++i) {
        int value = hist.at<int>(0, i);
        if (value != 0) {
            min = i;
            break;
        }
    }
    for (int i = 255; i >= 0; --i) {
        int value = hist.at<int>(0, i);
        if (value != 0) {
            max = i;
            break;
        }
    }
    long sum = 0, total = 0;
    for (int i = 0; i < 256; ++i) {
        int value = hist.at<int>(0, i);
        total += i;
        sum += value * i;
    }
    int avg = static_cast<int>(sum / (r_pixel * c_pixel));

    INFO("average gray " + std::to_string(avg));
    INFO("  image size " + std::to_string(c_pixel) + " x " + std::to_string(r_pixel));
    INFO("minimum gray " + std::to_string(min));
    INFO("maximum gray " + std::to_string(max));

    return 0;
}
