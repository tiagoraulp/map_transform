#ifndef VIS_TRANSF_HPP
#define VIS_TRANSF_HPP

#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/core/core.hpp>
#include <mutex>



template <typename T>
class Vis_transf{
protected:
    ros::NodeHandle nh_;
    ros::Publisher pub;
    ros::Publisher pub2;
    ros::Publisher pub3;
    ros::Publisher pub4;
    ros::Publisher pub5;
    ros::Publisher pub6;
    ros::Subscriber sub;
    dynamic_reconfigure::Server<T> server;
    typename dynamic_reconfigure::Server<T>::CallbackType func;
    tf::TransformListener pos_listener;
    int count;
    bool pos_rcv;
    bool treated, treated2;
    int infl;
    int defl;
    cv::Point2i prev;
    std::string tf_pref;
    int height, width;
    float res, or_x, or_y;
    bool _debug, gt, gt_c, changed, changed2;
    double rxr,ryr;
    double scale;
    bool changed_p;
    T _config;
    nav_msgs::OccupancyGrid msg_rcv,msg_rcv_pub;
    cv::Mat cv_map, map_or, map_erosionOp, map_closeOp, map_label , map_act, map_vis, map_debug, map_truth, map_erosionOpPrintColor;

    nav_msgs::OccupancyGrid Mat2RosMsg(cv::Mat map ,const nav_msgs::OccupancyGrid& msg);
    void rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void callbackParameters(T &config, uint32_t level);
    bool getTFPosition(cv::Point2d&p);
    bool getPosition(cv::Point2i&pos);
    bool checkProceed(void);
    bool checkProceed2(void);
    bool reachability_map(std::vector<std::vector<cv::Point> > labels, cv::Point2i pos, cv::Mat & r_map);
    std::vector<cv::Point> expVisibility_obs(cv::Point2i crit, int defl, cv::Mat regions, uchar k, std::vector<float> extremes, unsigned obt_angle, cv::Mat &vis_map_temp);
    std::vector<cv::Point> getExtremeFromObstacles(std::vector<cv::Point> occ, cv::Point2i crit);
public:
    Vis_transf(ros::NodeHandle nh);
    virtual ~Vis_transf();
    void update(void);
    void show(void);
    void publish(void);
    void transf(void);
    void transf_pos(void);
};

extern std::mutex mtx;




#endif // VIS_TRANSF_HPP
