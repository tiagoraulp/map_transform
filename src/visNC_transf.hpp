#ifndef VISNC_TRANSF_HPP
#define VISNC_TRANSF_HPP

#include "vis_transf.hpp"

#include "morph.hpp"



template <typename T>
class VisNC_transf: public Vis_transf<T>{
protected:
    cv::Mat robot;

    double rinfl;

    double sdefl;

    double rtr, rcx, rcy, rct;

    int angle_res;

    double angle_debug;

    Elem robot_or;

    nav_msgs::OccupancyGrid msg_rcv,msg_rcv_pub;

    cv::Mat struct_elem;

public:
    VisNC_transf(ros::NodeHandle nh, cv::Mat rob);
    virtual ~VisNC_transf();

    void update(void);
    void show(void);
    void transf(void);
    void transf_pos(void);


};


#endif // VISNC_TRANSF_HPP
