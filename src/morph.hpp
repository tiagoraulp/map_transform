#ifndef MORPH_H
#define MORPH_H

#include <opencv2/core/core.hpp>


class Elem{
public:
    std::vector<cv::Mat> elems;
    cv::Point pt;
    int  pu, pb, pl, pr;
};


Elem multiElem(cv::Mat elem, cv::Point2f pt, double orient ,double scale, int res);

std::vector<cv::Mat> multiErosion(cv::Mat map, Elem robot_or);

std::vector<cv::Mat> multiDilation(std::vector<cv::Mat> map_er, Elem robot_or );

Elem multiMerge(Elem robot_or,Elem sensor_or);


#endif // MORPH_H
