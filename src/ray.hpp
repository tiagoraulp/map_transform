#ifndef RAY_HPP
#define RAY_HPP

#include <opencv2/core/core.hpp>
#include "morph.hpp"

void raytracing(cv::Mat *map, cv::Point2i opt, cv::Point2i ref, cv::Point2i dest, float dist_t);

template <typename T, typename T2>
cv::Mat brute_force(cv::Mat map, T reach, T2 defl, bool opt=true, cv::Mat act=cv::Mat(0,0,CV_8UC1));


#endif // RAY_HPP
