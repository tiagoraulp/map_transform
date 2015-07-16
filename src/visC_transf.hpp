#ifndef VISC_TRANSF_HPP
#define VISC_TRANSF_HPP

#include "vis_transf.hpp"
#include "map_transform/ParametersConfig.h"
#include "unreachable.hpp"

class VisC_transf: public Vis_transf<map_transform::ParametersConfig>{
protected:
    int infl;
    int defl;

    bool reachability_map(cv::Point2i pos, cv::Mat & r_map);
    std::vector<cv::Point> expVisibility_obs(cv::Point2i crit, int defl, cv::Mat regions, uchar k, std::vector<float> extremes, unsigned obt_angle, cv::Mat &vis_map_temp);
    std::vector<cv::Point> getExtremeFromObstacles(std::vector<cv::Point> occ, cv::Point2i crit);
    virtual cv::Mat ext_vis(Unreachable, cv::Mat, cv::Mat);
    virtual void update_config(map_transform::ParametersConfig config);
    virtual void show(void);
    virtual void conf_space(void);
    virtual void visibility(cv::Point2i, bool, ros::Time);
public:
    VisC_transf(ros::NodeHandle nh);
    virtual ~VisC_transf();
};


#endif // VISC_TRANSF_HPP
