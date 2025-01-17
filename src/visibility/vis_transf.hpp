#ifndef VIS_TRANSF_HPP
#define VIS_TRANSF_HPP

#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/core/core.hpp>
//#include <mutex>

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
    ros::Publisher act_dist_pub;
    dynamic_reconfigure::Server<T> server;
    typename dynamic_reconfigure::Server<T>::CallbackType func;
    tf::TransformListener pos_listener;
    int count;
    bool resCS;
    bool pos_rcv;
    bool treated, treated2;
    cv::Point3i prev;
    std::string tf_pref;
    int height, width;
    float res, or_x, or_y;
    bool _debug, _show, gt, gt_c, changed, changed2;
    bool pub_once, frga;
    bool city;
    bool two_maps;
    double rxr,ryr, rtr;
    bool changed_p;
    double map_scale;
    T _config;
    nav_msgs::OccupancyGrid msg_rcv,msg_rcv_pub;
    cv::Mat cv_map_perc, cv_map_scaled_perc, map_or_perc, cv_map, cv_map_scaled, map_or, map_erosionOp, map_closeOp, map_label , map_act, map_vis, map_debug,
            map_truth, map_erosionOpPrintColor, map_erosionOpSmall, map_labelSmall, map_comp;
    std::vector<cv::Point> small_frontiers;
    cv::Mat_<int> act_dist;

    std::vector<cv::Point> getExtremeFromObstacles(std::vector<cv::Point> occ, cv::Point2i crit);
    nav_msgs::OccupancyGrid Mat2RosMsg(cv::Mat map ,const nav_msgs::OccupancyGrid& msg);
    void rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    virtual void callbackParameters(T &config, uint32_t level);
    virtual bool getTFPosition(cv::Point3d&p);
    virtual void getPosition(cv::Point3d&p);
    virtual void get2DPosition(cv::Point3i&pos, cv::Point3d p);
    virtual bool getPos(cv::Point3i&pos);
    virtual bool checkProceed(void);
    virtual bool checkProceed2(void);
    virtual void update(bool _opt);
    virtual void update_config(T config, bool ch, bool _opt)=0;
    virtual void show(void)=0;
    virtual void publish(void);
    virtual void transf(void);
    virtual void transf_pos(void);
    virtual bool conf_space(void)=0;
    virtual bool valid_pos(cv::Point3i pos)=0;
    virtual void visibility(cv::Point3i, bool, ros::Time)=0;
    virtual void clearImgs(void)=0;
public:
    Vis_transf(ros::NodeHandle nh);
    virtual ~Vis_transf();

    virtual void run(bool _opt=true);
};

//extern std::mutex mtx;




#endif // VIS_TRANSF_HPP
