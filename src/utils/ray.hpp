#ifndef RAY_HPP
#define RAY_HPP

#include <opencv2/core/core.hpp>
#include <vector>

void raytracing(cv::Mat *map, cv::Point2i opt, cv::Point2i ref, cv::Point2i dest, float dist_t);

bool raytracing(cv::Mat map, int opt_x, int opt_y, int dest_x, int dest_y, bool t);

bool raytracing(cv::Mat map, int opt_x, int opt_y, int dest_x, int dest_y, cv::Mat & test_pt, std::vector<cv::Point> & list);

bool raytracing(cv::Mat map, int opt_x, int opt_y, int dest_x, int dest_y, bool t, cv::Mat & test_pt, std::vector<cv::Point> & list);

#endif // RAY_HPP
