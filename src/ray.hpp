#ifndef RAY_HPP
#define RAY_HPP

#include <opencv2/core/core.hpp>
#include "morph.hpp"

void raytracing(cv::Mat *map, cv::Point2i opt, cv::Point2i ref, cv::Point2i dest, float dist_t);

bool raytracing(cv::Mat map, int opt_x, int opt_y, int dest_x, int dest_y, bool t);

template <typename T, typename T2>
cv::Mat brute_force(cv::Mat map, T reach, T2 defl, bool opt=true, cv::Mat act=cv::Mat(0,0,CV_8UC1), bool opt_rep=false);

template <typename T2>
cv::Mat bf_pt(cv::Mat map, cv::Point pt, T2 defl, cv::Mat vis, bool opt_rep=false);

std::vector<cv::Point> bf_hlx(Elem sensor);

std::vector<cv::Point> bf_hlx(int defl);

float coverage(cv::Mat test, cv::Mat truth);

std::vector<int> confusion_matrix(cv::Mat test, cv::Mat truth);

#endif // RAY_HPP
