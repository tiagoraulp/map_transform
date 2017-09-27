#ifndef COLOR_HPP
#define COLOR_HPP

#include <opencv2/core/core.hpp>

#define OPEN_COLOR 85
#define CLOSED_COLOR 170
#define GOALS_COLOR 250
#define PERC_COLOR 255

cv::Mat printPoint(cv::Mat img, cv::Point pos, unsigned char* color);

cv::Mat color_print(cv::Mat img1, cv::Mat img2, unsigned char* c_b, unsigned char* c_n, unsigned char* c_1, unsigned char* c_2);

cv::Mat color_print3(cv::Mat img1, cv::Mat img2, cv::Mat img3, unsigned char* c_123, unsigned char* c_12, unsigned char* c_13, unsigned char* c_23, unsigned char* c_1, unsigned char* c_2, unsigned char* c_3, unsigned char* c_0);

cv::Mat color_print_expansion(cv::Mat img0, cv::Mat img1, cv::Mat img2, cv::Point target, unsigned char* c_b, unsigned char* c_w, unsigned char* c_n, unsigned char* c_o, unsigned char* c_c, unsigned char* c_g, unsigned char* c_p, unsigned char* c_t);

#endif // COLOR_HPP
