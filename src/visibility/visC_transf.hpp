#ifndef VISC_TRANSF_HPP
#define VISC_TRANSF_HPP

#include "vis_transf.hpp"
#include "map_transform/ParametersConfig.h"
#include "unreachable.hpp"
#include "map_transform/VisCom.h"
#include "map_transform/VisNode.h"


class VisC_transf: public Vis_transf<map_transform::ParametersConfig>{
protected:
    int infl;
    int defl;
    ros::Publisher  graph_publisher;
    bool opt;
    bool fast_opt;
    std::vector<cv::Point2i> crit_pts;
    cv::Mat vis_temp, vis_map;

    std::vector<cv::Point> expVisibility_obs(cv::Point2i crit, int defl, cv::Mat regions, uchar k, std::vector<float> extremes, unsigned obt_angle, cv::Mat &vis_map_temp, std::vector<cv::Point> &vis_map_temp_list);
    bool reachability_map(cv::Point3i pos, cv::Mat & r_map);
    cv::Mat ext_vis(Unreachable, Unreachable, cv::Mat r_map, bool _opt=true);
    void update_config(map_transform::ParametersConfig config, bool ch, bool _opt);
    void show(void);
    bool conf_space(void);
    void visibility(cv::Point3i, bool, ros::Time);
    bool valid_pos(cv::Point3i pos);
    void clearImgs(void);
    void publish(void);
    map_transform::VisCom vis_;
    //std::vector<float> vis_;
public:
    VisC_transf(ros::NodeHandle nh);
    ~VisC_transf();
};


#endif // VISC_TRANSF_HPP
