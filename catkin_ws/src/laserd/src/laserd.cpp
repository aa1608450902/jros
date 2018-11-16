#include <laserd/laserd.h>
/// google log
#include <glog/logging.h>
/// opencv2
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <laserd/laserd.h>
#include <ros/ros.h>

namespace jlib {

cv::Mat laserd::filter(const cv::Mat &src) {
    cv::Mat dst;
    cv::GaussianBlur(src, dst, cv::Size(5, 5), 0);
    return dst;
}

cv::Mat laserd::calcGrayHist(const cv::Mat &img) {
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

cv::Mat laserd::seperate(const cv::Mat &src, double factor) {
    int rows = src.rows, cols = src.cols;
    cv::Mat inte = src;
    cv::Mat hist = calcGrayHist(inte);
    int minGray = 0, maxGray = 255;
    for (int i = 0; i < hist.cols; i++) {
        if (int(hist.at<uchar>(0, i)) > 0) {
            minGray = i;
            break;
        }
    }
    for (int i = hist.cols - 1; i >= 0; i--) {
        if (int(hist.at<uchar>(0, i)) > 0) {
            maxGray = i;
            break;
        }
    }
    int avgGray = minGray + int((maxGray - minGray) * factor);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (inte.at<uchar>(r, c) > avgGray)
                inte.at<uchar>(r, c) = 255;
            else
                inte.at<uchar>(r, c) = 0;
        }
    }
    return inte;
}

int laserd::calcCenterLaser(const cv::Mat &binary_image, cv::Point& center) {
    int x_sum = 0, y_sum = 0;
    int x_count = 0, y_count = 0;
    for (int r = 0; r < binary_image.rows; ++r) {
        for (int c = 0; c < binary_image.cols; ++c) {
            if (int(binary_image.at<uchar>(r, c)) == 255) {
                x_count++; x_sum += c;
                y_count++; y_sum += r;
            }
        }
    }
    if (!x_count || !y_count) {
        // LOG(INFO) << "x_count: " << x_count << " ; x_sum: " << x_sum;
        // LOG(INFO) << "y_count: " << y_count << " ; y_sum: " << y_sum;
        return -1;
    }
    center.x = int(x_sum / x_count - binary_image.cols / 2);
    center.y = int(binary_image.rows / 2 - y_sum / y_count);
    return 0;
}

int laserd::calcCenterLaserO(const cv::Mat &binary_image, cv::Point& center) {
	int x_sum = 0, y_sum = 0;
	int x_count = 0, y_count = 0;
	for (int r = 0; r < binary_image.rows; ++r) {
		for (int c = 0; c < binary_image.cols; ++c) {
			if (int(binary_image.at<uchar>(r, c)) > 0) {
				x_count++; x_sum += c;
				y_count++; y_sum += r;
			}
		}
	}
	if (!x_count || !y_count) {
		return -1;
	}
	center.x = int(x_sum / x_count - binary_image.cols / 2);
	center.y = int(binary_image.rows / 2 - y_sum / y_count);
	return 0;
}

int laserd::searchPeakDomain(const cv::Mat &src) {
    cv::Mat img(src);

    std::set<std::pair<int, int>> knownArea;
    int areaCount = 0;
    int rows = img.rows, cols = img.cols;
    for (int r = 0; r < rows - 1; ++r) {
        for (int c = 0; c < cols - 1; ++c) {
            std::pair<int, int> p(r, c);
            if (int(src.at<uchar>(p.first, p.second)) == 255 && !hasPointInSet(knownArea, p)) {
                areaCount++;
                knownArea.insert(p);
                expandDomain(img, p, knownArea);
            }
        }
    }
    return areaCount;
}

void
laserd::expandDomain(const cv::Mat &map, std::pair<int, int> start, std::set<std::pair<int, int>> &collection) {
    std::set<std::pair<int, int>> border{start};
    bool doSearch = true;
    while (doSearch) {
        std::set<std::pair<int, int>> newBorder;
        for (auto& pot : border) {
            if (pot.first == 0 && pot.second == 0) {
                /// only right - down
                std::pair<int, int>    up(pot.first, pot.second + 1); // right
                std::pair<int, int> right(pot.first + 1, pot.second); // down
                if (int(map.at<uchar>(up.first, up.second)) == 255 && !hasPointInSet(collection, up)) {
                    newBorder.insert(up);
                    collection.insert(up);
                }
                if (int(map.at<uchar>(right.first, right.second)) == 255 && !hasPointInSet(collection, right)) {
                    newBorder.insert(right);
                    collection.insert(right);
                }
            } else if (pot.first == 0 && pot.second == map.cols - 1) {
                /// no up - right, only left - down
                std::pair<int, int>  down(pot.first, pot.second - 1); // left
                std::pair<int, int> right(pot.first + 1, pot.second); // down
                if (int(map.at<uchar>(down.first, down.second)) == 255 && !hasPointInSet(collection, down)) {
                    newBorder.insert(down);
                    collection.insert(down);
                }
                if (int(map.at<uchar>(right.first, right.second)) == 255 && !hasPointInSet(collection, right)) {
                    newBorder.insert(right);
                    collection.insert(right);
                }
            } else if (pot.first == 0) {
                /// no up
                std::pair<int, int>    up(pot.first, pot.second + 1); // right
                std::pair<int, int>  down(pot.first, pot.second - 1); // left
                std::pair<int, int> right(pot.first + 1, pot.second); // down
                if (int(map.at<uchar>(up.first, up.second)) == 255 && !hasPointInSet(collection, up)) {
                    newBorder.insert(up);
                    collection.insert(up);
                }
                if (int(map.at<uchar>(down.first, down.second)) == 255 && !hasPointInSet(collection, down)) {
                    newBorder.insert(down);
                    collection.insert(down);
                }
                if (int(map.at<uchar>(right.first, right.second)) == 255 && !hasPointInSet(collection, right)) {
                    newBorder.insert(right);
                    collection.insert(right);
                }
            } else if (pot.first == map.rows - 1 && pot.second == 0) {
                /// no left - down
                std::pair<int, int>    up(pot.first, pot.second + 1); // right
                std::pair<int, int>  left(pot.first - 1, pot.second); // up
                if (int(map.at<uchar>(up.first, up.second)) == 255 && !hasPointInSet(collection, up)) {
                    newBorder.insert(up);
                    collection.insert(up);
                }
                if (int(map.at<uchar>(left.first, left.second)) == 255 && !hasPointInSet(collection, left)) {
                    newBorder.insert(left);
                    collection.insert(left);
                }
            } else if (pot.first == map.rows - 1 && pot.second == map.cols - 1) {
                /// no right - down
                std::pair<int, int>  down(pot.first, pot.second - 1); // left
                std::pair<int, int>  left(pot.first - 1, pot.second); // up
                if (int(map.at<uchar>(down.first, down.second)) == 255 && !hasPointInSet(collection, down)) {
                    newBorder.insert(down);
                    collection.insert(down);
                }
                if (int(map.at<uchar>(left.first, left.second)) == 255 && !hasPointInSet(collection, left)) {
                    newBorder.insert(left);
                    collection.insert(left);
                }
            } else if (pot.first == map.rows - 1) {
                /// no down
                std::pair<int, int>    up(pot.first, pot.second + 1); // right
                std::pair<int, int>  down(pot.first, pot.second - 1); // left
                std::pair<int, int>  left(pot.first - 1, pot.second); // up
                if (int(map.at<uchar>(up.first, up.second)) == 255 && !hasPointInSet(collection, up)) {
                    newBorder.insert(up);
                    collection.insert(up);
                }
                if (int(map.at<uchar>(down.first, down.second)) == 255 && !hasPointInSet(collection, down)) {
                    newBorder.insert(down);
                    collection.insert(down);
                }
                if (int(map.at<uchar>(left.first, left.second)) == 255 && !hasPointInSet(collection, left)) {
                    newBorder.insert(left);
                    collection.insert(left);
                }
            } else {
                std::pair<int, int>    up(pot.first, pot.second + 1); // right
                std::pair<int, int>  down(pot.first, pot.second - 1); // left
                std::pair<int, int>  left(pot.first - 1, pot.second); // up
                std::pair<int, int> right(pot.first + 1, pot.second); // down
                if (int(map.at<uchar>(up.first, up.second)) == 255 && !hasPointInSet(collection, up)) {
                    newBorder.insert(up);
                    collection.insert(up);
                }
                if (int(map.at<uchar>(down.first, down.second)) == 255 && !hasPointInSet(collection, down)) {
                    newBorder.insert(down);
                    collection.insert(down);
                }
                if (int(map.at<uchar>(left.first, left.second)) == 255 && !hasPointInSet(collection, left)) {
                    newBorder.insert(left);
                    collection.insert(left);
                }
                if (int(map.at<uchar>(right.first, right.second)) == 255 && !hasPointInSet(collection, right)) {
                    newBorder.insert(right);
                    collection.insert(right);
                }
            }
        }

        border.swap(newBorder);

        if (border.empty()) {
            doSearch = false;
        }
    }
}

bool laserd::hasPointInSet(const std::set<std::pair<int, int>> &collection, const std::pair<int, int> &point) {
    auto it = collection.cbegin();
    while (it != collection.cend()) {
        if (it->first == point.first && it->second == point.second)
            return true;
        it++;
    }
    return false;
}

int laserd::searchRedLaser(const cv::Mat& image, cv::Point& __r) {
    int width = image.rows, height = image.rows;
	// __r.x = int(width / 2);
 //    __r.y = int(height / 2);
    __r.x = 0; __r.y = 0;
    int channels = image.channels();

    if (channels == 1) {
        // LOG(INFO) << "image channels: " << channels;
    } else if (channels == 3) {
        // LOG(INFO) << "image channels: " << channels;
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

        cv::Mat red_mask_1, red_mask_2, red_mask;
	    std::vector<int> red_hsv_1_low{0, 180, 100}, red_hsv_1_top{22, 255, 255},
	                     red_hsv_2_low{160, 180, 100}, red_hsv_2_top{180, 255, 255};
	    red_hsv_1_low = ros::param::param<std::vector<int>>("hsv_green_low", red_hsv_1_low);
	    red_hsv_1_top = ros::param::param<std::vector<int>>("hsv_green_low", red_hsv_1_top);
	    red_hsv_2_low = ros::param::param<std::vector<int>>("hsv_green_low", red_hsv_2_low);
	    red_hsv_2_top = ros::param::param<std::vector<int>>("hsv_green_low", red_hsv_2_top);

        cv::inRange(hsv_image, cv::Scalar(red_hsv_1_low[0], red_hsv_1_low[1], red_hsv_1_low[2]),
        		cv::Scalar(red_hsv_1_top[0], red_hsv_1_top[1], red_hsv_1_top[2]), red_mask_1);
        cv::inRange(hsv_image, cv::Scalar(red_hsv_2_low[0], red_hsv_2_low[1], red_hsv_2_low[2]),
        		cv::Scalar(red_hsv_2_top[0], red_hsv_2_top[1], red_hsv_2_top[2]), red_mask_2);
        cv::bitwise_or(red_mask_1, red_mask_2, red_mask);

        int closing_operation_count = 1;
        cv::Mat iterate_image(red_mask);
        while (closing_operation_count <= ros::param::param<int>("red_mask_close_times", 3)) {
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                    cv::Size(2 * closing_operation_count + 1, 2 * closing_operation_count + 1));
            cv::morphologyEx(iterate_image, iterate_image, cv::MORPH_CLOSE, kernel);
            closing_operation_count++;
        }

        cv::Mat closing_image;
        cv::bitwise_and(iterate_image, red_mask, closing_image);
        cv::Mat target = jlib::laserd::seperate(closing_image, ros::param::param<double>("red_mask_seperate_ratio", 0.8));
        /// ---------------------------------------------------------
        cv::vector<cv::Mat> triple_channels;
        split(image, triple_channels);
        cv::Mat red_gray_image = triple_channels[2];
        cv::Mat gauss_image;
        cv::GaussianBlur(red_gray_image, gauss_image, cv::Size(5, 5), 0);
        cv::Mat target0 = jlib::laserd::seperate(gauss_image, ros::param::param<double>("red_gray_seperate_ratio", 0.8));
        /// ---------------------------------------------------------
        cv::Mat final_image;
        cv::bitwise_and(target0, target, final_image);
        /// ---------------------------------------------------------
        closing_operation_count = 1;
        iterate_image = final_image;
        while (closing_operation_count <= ros::param::param<int>("red_final_close_times", 2)) {
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                                                       cv::Size(2 * closing_operation_count + 1, 2 * closing_operation_count + 1));
            cv::morphologyEx(iterate_image, iterate_image, cv::MORPH_CLOSE, kernel);
            closing_operation_count++;
        }
        final_image = iterate_image;
        int domain = jlib::laserd::searchPeakDomain(final_image);
        if (domain == 0) {
            return -1;
        } else if (domain == 2) {
            return -1;
        }
        if (!jlib::laserd::calcCenterLaser(final_image, __r)) {
            return 0;
        }
        // cv::imshow("target", final_image);
        // cv::waitKey(0);
    } else {
        // LOG(ERROR) << "error image channels: " << channels;
    }
    return -1;
}

int laserd::searchGreenLaser(const cv::Mat& image, cv::Point& __r) {
	int width = image.rows, height = image.rows;
    // __r.x = int(width / 2);
    // __r.y = int(height / 2);
    __r.x = 0; __r.y = 0;
	int channels = image.channels();

	if (channels == 1) {
		// LOG(INFO) << "image channels: " << channels;
	} else if (channels == 3) {
		// LOG(INFO) << "image channels: " << channels;
		cv::Mat hsv_image;
		cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

		cv::Mat green_mask;
		std::vector<int> green_hsv_low{50, 150, 100}, green_hsv_top{70, 255, 255};
		green_hsv_low = ros::param::param<std::vector<int>>("hsv_green_low", green_hsv_low);
		green_hsv_top = ros::param::param<std::vector<int>>("hsv_green_low", green_hsv_top);
//		cv::inRange(hsv_image, cv::Scalar(50, 150, 100), cv::Scalar(70, 255, 255), green_mask);
		cv::inRange(hsv_image, cv::Scalar(green_hsv_low[0], green_hsv_low[1], green_hsv_low[2]),
				cv::Scalar(green_hsv_top[0], green_hsv_top[1], green_hsv_top[2]), green_mask);

		int closing_operation_count = 1;
		cv::Mat iterate_image(green_mask);
		while (closing_operation_count <= ros::param::param<int>("green_mask_close_times", 3)) {
			cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
			                                           cv::Size(2 * closing_operation_count + 1, 2 * closing_operation_count + 1));
			cv::morphologyEx(iterate_image, iterate_image, cv::MORPH_CLOSE, kernel);
			closing_operation_count++;
		}
        /// ---------------------------------------------------------
		cv::Mat closing_image;
		cv::bitwise_and(iterate_image, green_mask, closing_image);
		cv::Mat target = jlib::laserd::seperate(closing_image, ros::param::param<double>("green_mask_seperate_ratio", 0.8));
		/// ---------------------------------------------------------
		cv::vector<cv::Mat> triple_channels;
		split(image, triple_channels);
		cv::Mat red_gray_image = triple_channels[2];
		cv::Mat gauss_image;
		cv::GaussianBlur(red_gray_image, gauss_image, cv::Size(5, 5), 0);
		cv::Mat target0 = jlib::laserd::seperate(gauss_image, ros::param::param<double>("green_gray_seperate_ratio", 0.8));
		/// ---------------------------------------------------------
		cv::Mat final_image;
		cv::bitwise_and(target0, target, final_image);
		/// ---------------------------------------------------------
		closing_operation_count = 1;
		iterate_image = final_image;
		while (closing_operation_count <= ros::param::param<int>("green_final_close_times", 3)) {
			cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
			                                           cv::Size(2 * closing_operation_count + 1, 2 * closing_operation_count + 1));
			cv::morphologyEx(iterate_image, iterate_image, cv::MORPH_CLOSE, kernel);
			closing_operation_count++;
		}
		final_image = iterate_image;

		int domain = jlib::laserd::searchPeakDomain(final_image);
		if (domain == 0) {
			return -1;
		} else if (domain == 2) {
			return -1;
		}
		if (!jlib::laserd::calcCenterLaser(final_image, __r)) {
			return 0;
		}
	} else {
		// LOG(ERROR) << "error image channels: " << channels;
	}
	return -1;
}

	int laserd::searchGreenLaserO(const cv::Mat& image, cv::Point& __r) {
		int width = image.rows, height = image.rows;
		__r.x = 0;
		__r.y = 0;
		int channels = image.channels();

		if (channels == 1) {
			// LOG(INFO) << "image channels: " << channels;
		} else if (channels == 3) {
			// LOG(INFO) << "image channels: " << channels;
			cv::Mat hsv_image;
			cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

			cv::Mat green_mask;
			std::vector<int> green_hsv_low{35, 150, 200}, green_hsv_top{77, 255, 255};
			green_hsv_low = ros::param::param<std::vector<int>>("hsv_green_low", green_hsv_low);
			green_hsv_top = ros::param::param<std::vector<int>>("hsv_green_low", green_hsv_top);
//		cv::inRange(hsv_image, cv::Scalar(50, 150, 100), cv::Scalar(70, 255, 255), green_mask);
			cv::inRange(hsv_image, cv::Scalar(green_hsv_low[0], green_hsv_low[1], green_hsv_low[2]),
			            cv::Scalar(green_hsv_top[0], green_hsv_top[1], green_hsv_top[2]), green_mask);
            //cv::imshow("green", green_mask);
            //cv::waitKey(0);
            //ROS_INFO("-------------1---------");
			if (!jlib::laserd::calcCenterLaserO(green_mask, __r)) {
				return 0;
			}
		} else {
			// LOG(ERROR) << "error image channels: " << channels;
		}
        // ROS_INFO("RETURN -1");
		return -1;
	}

}