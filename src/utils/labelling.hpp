#ifndef LABELLING_HPP
#define LABELLING_HPP

#include <opencv2/core/core.hpp>

std::vector<std::vector<cv::Point> >  label(const cv::Mat binary, int conn);
std::vector<cv::Point> label_seed(const cv::Mat binary, int conn, cv::Point seed);

#endif // LABELLING_HPP
