#ifndef VIS_TRANSF_HPP
#define VIS_TRANSF_HPP

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <map_transform/ParametersConfig.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <thread>
#include <mutex>

#include "vector_utils.hpp"
#include "labelling.hpp"
#include "unreachable.hpp"
#include "CritPoints.hpp"
#include "clustering.hpp"
#include "bugfollowing.hpp"
#include "chain.hpp"
#include "obs_extremes.hpp"
#include "ray.hpp"

using namespace std;

mutex mtx;

const double PI = 3.141592653589793;

static const std::string M_WINDOW = "Map";
static const std::string E_WINDOW = "Erosion";
static const std::string C_WINDOW = "Close";
static const std::string L_WINDOW = "Labelled";
static const std::string A_WINDOW = "Actuation";
static const std::string V_WINDOW = "Visibility";
static const std::string D_WINDOW = "Debug";
static const std::string G_WINDOW = "Ground_Truth";

class Vis_transf{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub;
    ros::Publisher pub2;
    ros::Publisher pub3;
    ros::Publisher pub4;
    ros::Publisher pub5;
    ros::Publisher pub6;
    ros::Subscriber sub;
    dynamic_reconfigure::Server<map_transform::ParametersConfig> server;
    dynamic_reconfigure::Server<map_transform::ParametersConfig>::CallbackType func;
    tf::TransformListener pos_listener;
    int count;
    bool pos_rcv;
    bool treated, treated2;
    int infl;
    int defl;
    cv::Point2i prev;
    string tf_pref;
    int height, width;
    float res, or_x, or_y;
    bool _debug, gt, gt_c, changed, changed2;
    double rxr,ryr;
    bool changed_p;
    map_transform::ParametersConfig _config;
    nav_msgs::OccupancyGrid msg_rcv,msg_rcv_pub;
    cv::Mat cv_map, map_or, map_erosionOp, map_closeOp, map_label , map_act, map_vis, map_debug, map_truth, map_erosionOpPrintColor;

    nav_msgs::OccupancyGrid Mat2RosMsg(cv::Mat map ,const nav_msgs::OccupancyGrid& msg);
    void rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void callbackParameters(map_transform::ParametersConfig &config, uint32_t level);
    bool getTFPosition(cv::Point2d&p);
    bool getPosition(cv::Point2i&pos);
    bool checkProceed(void);
    bool checkProceed2(void);
    cv::Mat printPoint(cv::Mat img, cv::Point2i pos, unsigned char* color);
    bool reachability_map(std::vector<std::vector<cv::Point> > labels, cv::Point2i pos, cv::Mat & r_map);
    vector<cv::Point> expVisibility_obs(cv::Point2i crit, int defl, cv::Mat regions, uchar k, vector<float> extremes, unsigned obt_angle, cv::Mat &vis_map_temp);
    vector<cv::Point> getExtremeFromObstacles(vector<cv::Point> occ, cv::Point2i crit);
public:
    Vis_transf(ros::NodeHandle nh);
    ~Vis_transf();
    void update(void);
    void show(void);
    void publish(void);
    void transf(void);
    void transf_pos(void);
};


#endif // VIS_TRANSF_HPP
