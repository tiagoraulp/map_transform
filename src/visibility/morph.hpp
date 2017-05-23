#ifndef MORPH_H
#define MORPH_H

#include <opencv2/core/core.hpp>


class Elem{
public:
    std::vector<cv::Mat> elems;
    cv::Point pt;
    std::vector <cv::Point> pt2;
    int  pu, pb, pl, pr;
};


Elem multiSensor(cv::Mat elem, cv::Point2f pt, double orient ,double scale, int res, cv::Point2f pt2);

Elem multiRobot(cv::Mat elem, cv::Point2f& pt, double orient ,double scale, int res, cv::Point2f& pt2);

std::vector<cv::Mat> multiErosion(cv::Mat map, Elem robot_or);

std::vector<cv::Mat> multiDilation(std::vector<cv::Mat> map_er, Elem robot_or );

void multiMerge(Elem robot_or,Elem& sensor_or, Elem& result, Elem& rev);


#endif // MORPH_H
