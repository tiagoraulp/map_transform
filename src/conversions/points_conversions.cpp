#include "points_conversions.hpp"

PointI convertW2I(geometry_msgs::Point p, double res){
    return PointI(round(p.x/res-0.5),round(p.y/res-0.5));
}

geometry_msgs::Point convertI2W(PointI p, double res){
    geometry_msgs::Point pf;
    pf.x=(p.i+0.5)*res;
    pf.y=(p.j+0.5)*res;
    return pf;
}

PointI convertWRobotPos2I(geometry_msgs::Point p, double res){
    return PointI(round(p.x/res),round(p.y/res));
}

geometry_msgs::Point convertI2W(cv::Point2i  p, double res){
    return convertI2W(PointI(p.x, p.y), res);
}
