#ifndef VISNC_TRANSF_HPP
#define VISNC_TRANSF_HPP

#include "vis_transf.hpp"
#include "map_transform/ParametersncConfig.h"
#include "morph.hpp"
#include "unreachable.hpp"

class VisNC_transf: public Vis_transf<map_transform::ParametersncConfig>{
protected:
    cv::Mat robot, sensor;
    double rinfl;
    double sdefl;
    double rcx, rcy, rct, scx, scy, sct, dx, dy;
    int angle_res, sens_res;
    double angle_debug;
    std::vector<int> sens_area;
    Elem robot_or, robot_act, sensor_or, sensor_ev;
    cv::Rect rec;
    cv::Mat struct_elemR, struct_elemS, struct_elemA, struct_elemEV, map_projLabel, map_projAct, map_projEros, map_projClose,map_debug_pos;
    std::vector<cv::Mat> multi_er_map, multi_labl_map, multi_act_map;
    bool opt;

    cv::Mat ext_vis(Unreachable unreach, cv::Mat vis_map, std::vector<cv::Mat> r_map, bool optRay=true);
    void update_config(map_transform::ParametersncConfig config, bool ch, bool _opt);
    void show(void);
    bool conf_space(void);
    void visibility(cv::Point3i, bool, ros::Time);
    void getPosition(cv::Point3d&p);
    void get2DPosition(cv::Point3i&pos, cv::Point3d p);
    bool valid_pos(cv::Point3i pos);
    void clearImgs();
    bool testConf();
    void calcSensArea(Elem sens);
public:
    VisNC_transf(ros::NodeHandle nh, cv::Mat rob, cv::Mat sens);
    virtual ~VisNC_transf();
};


#endif // VISNC_TRANSF_HPP
