#ifndef RAY_HPP
#define RAY_HPP

#include <opencv2/core/core.hpp>

void raytracing(cv::Mat *map, cv::Point2i opt, cv::Point2i ref, cv::Point2i dest, float dist_t);

cv::Mat brute_force(cv::Mat map, cv::Mat reach, int defl);

cv::Mat brute_force_opt(cv::Mat map, cv::Mat reach, int defl);

cv::Mat brute_force_opt_act(cv::Mat map, cv::Mat reach, cv::Mat act, int defl);


#endif // RAY_HPP
