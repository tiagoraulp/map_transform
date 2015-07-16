#include "vis_transf.hpp"


#include "vector_utils.hpp"
#include "color.hpp"

#include <map_transform/ParametersConfig.h>
#include <map_transform/ParametersncConfig.h>

using namespace std;

mutex mtx;

static const double PI = 3.141592653589793;

template <typename T>
void Vis_transf<T>::callbackParameters(T &config, uint32_t level) {
    mtx.lock();
    changed_p=true;
    _config=config;
    mtx.unlock();
}

template <typename T>
void Vis_transf<T>::update(void)
{
    bool proc=false;

    T config;

    mtx.lock();

    if(changed_p)
    {
        changed_p=false;
        config=_config;
        proc=true;
    }

    mtx.unlock();

    if(proc)
    {
        update_config(config);
    }
}


template <typename T>
Vis_transf<T>::Vis_transf(ros::NodeHandle nh): nh_(nh)
{
    pub = nh_.advertise<nav_msgs::OccupancyGrid>("e_map", 1,true);
    pub2 = nh_.advertise<nav_msgs::OccupancyGrid>("c_map", 1,true);
    pub3 = nh_.advertise<nav_msgs::OccupancyGrid>("r_map", 1,true);
    pub4 = nh_.advertise<nav_msgs::OccupancyGrid>("a_map", 1,true);
    pub5 = nh_.advertise<nav_msgs::OccupancyGrid>("v_map", 1,true);
    pub6 = nh_.advertise<nav_msgs::OccupancyGrid>("g_map", 1,true);
    sub = nh_.subscribe("map", 1, &Vis_transf::rcv_map, this);
    nh_.param("x", rxr, 50.0);
    nh_.param("y", ryr, 50.0);
    nh_.param("scale", scale, 100.0);
    scale/=100;

    nh_.param("tf_prefix", tf_pref, std::string(""));
    nh_.param("ground_truth", gt, false);
    nh_.param("debug", _debug, true);
    prev.x=-1; prev.y=-1;
    count=0;
    func = boost::bind(&Vis_transf::callbackParameters, this,_1, _2);
    server.setCallback(func);
    treated=false;
    treated2=true;
    pos_rcv=false;
    gt_c=false;
    changed=false;
    changed2=false;
    changed_p=false;


}

template <typename T>
Vis_transf<T>::~Vis_transf()
{
}



template <typename T>
nav_msgs::OccupancyGrid Vis_transf<T>::Mat2RosMsg(cv::Mat map ,const nav_msgs::OccupancyGrid& msg)
{
    //map=scaling(map, 1/scale);

    nav_msgs::OccupancyGrid n_msg;
    n_msg.header.stamp = ros::Time::now();
    n_msg.header.frame_id = msg.header.frame_id;
    n_msg.info=msg.info;

    n_msg.data.resize(msg.data.size(),-1);
    std::vector<signed char>::const_iterator mapDataIter = msg.data.begin();
    std::vector<signed char>::iterator n_mapDataIter = n_msg.data.begin();
    for(unsigned int i=0;i<msg.info.height;i++){
        for(unsigned int j=0;j<msg.info.width;j++){
                signed char val;
                if( *mapDataIter < 0)
                    val = -1;
                else if(map.at<unsigned char>(j,i) == 255)
                    val = 0;
                else
                    val = 100;

                *n_mapDataIter=val;

                n_mapDataIter++;
                mapDataIter++;
            }
    }
    return n_msg;
}

template <typename T>
void Vis_transf<T>::rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("I heard map: [%d]", msg->header.seq);

    cv::Mat prev_map=cv_map.clone();

    cv_map = cv::Mat(msg->info.width, msg->info.height, CV_8UC1);

    treated=true;

    res= msg->info.resolution;
    height= msg->info.height;
    width= msg->info.width;

    or_x= msg->info.origin.position.x;
    or_y= msg->info.origin.position.y;

    if(count>0)
    {
        if(cv_map.rows!=prev_map.rows || cv_map.cols!=prev_map.cols || res!=msg->info.resolution || or_x!=msg->info.origin.position.x || or_y!=msg->info.origin.position.y )
            treated=false;
    }
    else
        treated=false;


    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    signed char map_occ_thres = 90;
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
            unsigned char val_cv;
            if(*mapDataIterC >  map_occ_thres)
            {val_cv=0;}
            else if(*mapDataIterC == 0)
            {val_cv=255;}
            else
            {val_cv=255;}
            cv_map.at<uchar>(j,i) = val_cv;

            if(count>0 && treated)
                if(val_cv!=prev_map.at<uchar>(j,i))
                    treated=false;

            mapDataIterC++;
        }
    }

    ++count;

    msg_rcv=*msg;

    //cv_map=scaling(cv_map, scale);

    //res=res/scale;
    //height= cv_map.cols;
    //width= cv_map.rows;

    //or_x= msg->info.origin.position.x;
    //or_y= msg->info.origin.position.y;

    treated2=treated;
}

template <typename T>
bool Vis_transf<T>::checkProceed2(void)
{
    bool proc=false;

    if((!treated2) || changed2)
    {
        proc=true;
        treated2=true;
        changed2=false;
    }

    return proc;
}

template <typename T>
bool Vis_transf<T>::checkProceed(void)
{
    bool proc=false;

    if((!treated) || changed)
    {
        proc=true;
        treated=true;
        changed=false;
    }

    return proc;
}



template <typename T>
void Vis_transf<T>::transf(void)
{
    bool proc=checkProceed2();

    if(count>0 && (proc) )
    {
        ros::Time t01=ros::Time::now();

        conf_space();

        ros::Duration diff = ros::Time::now() - t01;

        ROS_INFO("%s - Time for configuration space: %f", tf_pref.c_str(), diff.toSec());
    }
}

template <typename T>
bool Vis_transf<T>::getTFPosition(cv::Point3d &p)
{
    tf::StampedTransform transform;
    try{
        pos_listener.lookupTransform("/map", tf_pref+"/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_INFO("%s",ex.what());
      return false;
    }
    p.x=transform.getOrigin().x();
    p.y=transform.getOrigin().y();
    p.z=tf::getYaw(transform.getRotation())/2/PI*360;
    return true;
}

template <typename T>
bool Vis_transf<T>::getPosition(cv::Point2i& pos, double& theta){
    cv::Point3d p;
    if(!_debug)
    {
        if(!getTFPosition(p))
            return false;
    }
    else
    {
        p.x=rxr/100.0*map_or.rows*res;
        p.y=ryr/100.0*map_or.cols*res;
        p.z=rtr;
    }

    if(p.z<0)
        p.z+=360;

    pos.x=(int) round((p.x-or_x)/res);
    pos.y=(int) round((p.y-or_y)/res);

    pos.x=boundPos(pos.x, map_or.rows);
    pos.y=boundPos(pos.y, map_or.cols);

    theta=p.z;

    return true;
}



template <typename T>
void Vis_transf<T>::transf_pos(void)
{
    if(count>0)
    {
        ros::Time t01=ros::Time::now();

        cv::Point2i pos;
        double theta;

        if (!getPosition(pos, theta))
            return;



        bool proc=checkProceed();

        if(map_erosionOp.at<uchar>(pos.x,pos.y)==0)  //invalid center position of the robot (touching obstacles or walls)
        {
            gt_c=false;

            pos_rcv=true;

            unsigned char color[3]={255,0,0};

            map_erosionOpPrintColor=printPoint(map_erosionOp, pos, color);

            map_label=cv::Mat::zeros(map_erosionOp.rows, map_erosionOp.cols, CV_8UC1);
            map_act=map_label;
            map_vis=map_label;
            map_debug=map_label;

            bool print=true;

            if (prev.x>=0 && prev.y>=0 && prev.x<map_erosionOp.rows && prev.y<map_erosionOp.cols)
                if (map_erosionOp.at<uchar>(prev.x,prev.y)==0 && !proc)
                {
                    print=false;
                }

            prev=pos;

            if(print)
            {
                ros::Duration diff = ros::Time::now() - t01;
                ROS_INFO("%s - Time for visibility (invalid position): %f", tf_pref.c_str(), diff.toSec());
            }

            return;
        }

        visibility(pos, proc, t01);

        pos_rcv=true;

        prev=pos;
    }
    else
        return;
}




template <typename T>
void Vis_transf<T>::publish(void)
{
    if(count>0)
    {
        nav_msgs::OccupancyGrid n_msg;

        n_msg=Mat2RosMsg(map_erosionOp , msg_rcv_pub);
        pub.publish(n_msg);

        n_msg=Mat2RosMsg(map_closeOp , msg_rcv_pub);
        pub2.publish(n_msg);

        if(pos_rcv)
        {
            n_msg=Mat2RosMsg( map_label , msg_rcv_pub);
            pub3.publish(n_msg);

            n_msg=Mat2RosMsg( map_act , msg_rcv_pub);
            pub4.publish(n_msg);

            n_msg=Mat2RosMsg( map_vis , msg_rcv_pub);
            pub5.publish(n_msg);

            if(gt  && gt_c)
            {
                n_msg=Mat2RosMsg( map_truth , msg_rcv_pub);
                pub6.publish(n_msg);
            }
        }
    }
}

template <typename T>
void Vis_transf<T>::run(void)
{
    update();

    transf();

    transf_pos();

    show();

    publish();
}

template class Vis_transf<map_transform::ParametersConfig>;
template class Vis_transf<map_transform::ParametersncConfig>;

