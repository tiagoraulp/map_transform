#ifndef BF_HPP
#define BF_HPP

#include <opencv2/core/core.hpp>
#include "morph.hpp"

template <typename T, typename T2>
cv::Mat brute_force(cv::Mat map, T reach, T2 defl, bool opt=true, cv::Mat act=cv::Mat(0,0,CV_8UC1), bool opt_rep=false, bool opt_repM=false);

template <typename T, typename T2>
cv::Mat bf_pt(cv::Mat map, T pt, T2 defl, cv::Mat vis, bool opt_rep=false, bool opt_repM=false);

std::vector<cv::Point> bf_hlx(Elem sensor);

std::vector<cv::Point> bf_hlx(int defl);

float coverage(cv::Mat test, cv::Mat truth);

std::vector<int> confusion_matrix(cv::Mat test, cv::Mat truth);

#endif // BF_HPP
