#ifndef COLOR_HPP
#define COLOR_HPP

#include <opencv2/core/core.hpp>

cv::Mat printPoint(cv::Mat img, cv::Point pos, unsigned char* color);

cv::Mat color_print(cv::Mat img1, cv::Mat img2, unsigned char* c_b, unsigned char* c_n, unsigned char* c_1, unsigned char* c_2);

cv::Mat color_print3(cv::Mat img1, cv::Mat img2, cv::Mat img3, unsigned char* c_123, unsigned char* c_12, unsigned char* c_13, unsigned char* c_23, unsigned char* c_1, unsigned char* c_2, unsigned char* c_3, unsigned char* c_0);

#endif // COLOR_HPP
