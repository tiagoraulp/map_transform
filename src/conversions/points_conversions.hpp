#ifndef POINTS_CONVERSIONS_HPP
#define POINTS_CONVERSIONS_HPP

#include "geometry_msgs/Point.h"
#include "pointi.hpp"
#include "opencv2/core/core.hpp"

PointI convertW2I(geometry_msgs::Point p, double res);
geometry_msgs::Point convertI2W(PointI p, double res);
PointI convertWtf2I(geometry_msgs::Point p, double res);

void convertW2I(geometry_msgs::Point p, double res, cv::Point2i & pt);
geometry_msgs::Point convertI2W(cv::Point2i p, double res);
geometry_msgs::Point convertI2W2D(cv::Point3i  p, double res);
void convertWtf2I(geometry_msgs::Point p, double res, cv::Point2i & pt);

#endif // POINTS_CONVERSIONS_HPP
