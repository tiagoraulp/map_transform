#ifndef POINTS_HPP
#define POINTS_HPP

#include "geometry_msgs/Point.h"

class PointI{
public:
    int i;
    int j;
    PointI(int a, int b);
    PointI();
};

PointI convertW2I(geometry_msgs::Point p, double res);
geometry_msgs::Point convertI2W(PointI p, double res);
PointI convertWRobotPos2I(geometry_msgs::Point p, double res);

#endif // POINTS_HPP
