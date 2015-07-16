#ifndef VISNC_TRANSF_HPP
#define VISNC_TRANSF_HPP

#include "vis_transf.hpp"
#include "map_transform/ParametersncConfig.h"

#include "morph.hpp"


class VisNC_transf: public Vis_transf<map_transform::ParametersncConfig>{
protected:
    cv::Mat robot;

    double rinfl;

    double sdefl;

    double rcx, rcy, rct;

    int angle_res;

    double angle_debug;

    Elem robot_or;

    cv::Mat struct_elem;

    void update_config(map_transform::ParametersncConfig config);
    void show(void);
    void conf_space(void);
    void visibility(cv::Point2i, bool, ros::Time);
    void getPosition(cv::Point3d&p);
    void get2DPosition(cv::Point2i&pos, double& theta, cv::Point3d p);
public:
    VisNC_transf(ros::NodeHandle nh, cv::Mat rob);
    virtual ~VisNC_transf();
};


#endif // VISNC_TRANSF_HPP
