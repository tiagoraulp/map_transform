#include "vis_transf.hpp"

#include "vector_utils.hpp"
#include "color.hpp"
#include "bugfollowing.hpp"
#include "obs_extremes.hpp"
#include "clustering.hpp"

#include "opencv2/imgproc/imgproc.hpp"

#include <std_msgs/Int16MultiArray.h>

#include <map_transform/ParametersConfig.h>
#include <map_transform/ParametersncConfig.h>

using namespace std;

//mutex mtx;

static const double PI = 3.141592653589793;

template <typename T>
void Vis_transf<T>::callbackParameters(T &config, uint32_t level) {
    //mtx.lock();
    changed_p=true;
    _config=config;
    //mtx.unlock();
}

template <typename T>
void Vis_transf<T>::update(bool _opt)
{
    bool proc=false;

    T config;

    //mtx.lock();

    bool prev_changed_p=false;

    if(changed_p || !_opt)
    {
        if(changed_p)
            prev_changed_p=true;
        changed_p=false;
        config=_config;
        proc=true;
    }

    //mtx.unlock();

    if(proc)
    {
        update_config(config, prev_changed_p,_opt);
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
    act_dist_pub = nh_.advertise<std_msgs::Int16MultiArray>("act_dist", 10,true);

    sub = nh_.subscribe("map", 1, &Vis_transf::rcv_map, this);
    nh_.param("x", rxr, 50.0);
    nh_.param("y", ryr, 50.0);
    rtr=0;

    nh_.param("tf_prefix", tf_pref, std::string(""));
    nh_.param("ground_truth", gt, false);
    nh_.param("debug", _debug, true);
    nh_.param("pub_once", pub_once, true);
    nh_.param("frga", frga, false);
    prev.x=-1; prev.y=-1; prev.z=-1;
    count=0;
    res=false;
    resCS=false;
    func = boost::bind(&Vis_transf::callbackParameters, this,_1, _2);
    server.setCallback(func);
    treated=false;
    treated2=true;
    pos_rcv=false;
    gt_c=false;
    changed=false;
    changed2=false;
    changed_p=false;

    map_scale=1;
    small_frontiers.clear();
}

template <typename T>
Vis_transf<T>::~Vis_transf()
{
}



template <typename T>
nav_msgs::OccupancyGrid Vis_transf<T>::Mat2RosMsg(cv::Mat map ,const nav_msgs::OccupancyGrid& msg)
{
    cv::resize(map, map, cv::Size(0,0), 1/map_scale, 1/map_scale, cv::INTER_NEAREST);

    nav_msgs::OccupancyGrid n_msg;
    n_msg.header.stamp = ros::Time::now();
    n_msg.header.frame_id = msg.header.frame_id;
    n_msg.info=msg.info;

    n_msg.data.assign(msg.data.size(),-1);
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
    //ROS_INFO("I heard map: [%d]", msg->header.seq);

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

    treated=false; // forces update whenever new map is received; delete if update should only run dynamically if map changes

    ++count;

    msg_rcv=*msg;

    treated2=treated;

    cv::resize(cv_map, cv_map_scaled, cv::Size(0,0), map_scale, map_scale, cv::INTER_NEAREST);
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

        resCS=conf_space();

        ros::Duration diff = ros::Time::now() - t01;

        if(!resCS)
        {
            ROS_INFO("%s - Time for configuration space (invalid configuration): %f", tf_pref.c_str(), diff.toSec());
        }
        else
        {
            ROS_INFO("%s - Time for configuration space: %f", tf_pref.c_str(), diff.toSec());
        }
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
    p.z=boundAngleD(p.z);
    return true;
}

template <typename T>
void Vis_transf<T>::getPosition(cv::Point3d &p)
{
        p.x=rxr/100.0*map_or.rows*res;
        p.y=ryr/100.0*map_or.cols*res;
        p.z=rtr;
}

template <typename T>
void Vis_transf<T>::get2DPosition(cv::Point3i& pos, cv::Point3d p)
{
    pos.x=(int) round((p.x-or_x)/res);
    pos.y=(int) round((p.y-or_y)/res);

    pos.x=boundPos(pos.x, map_or.rows);
    pos.y=boundPos(pos.y, map_or.cols);

    pos.z=angleD2I(p.z,1);
}

template <typename T>
bool Vis_transf<T>::getPos(cv::Point3i&pos)
{
    cv::Point3d p;
    if(!_debug)
    {
        if(!getTFPosition(p))
            return false;
    }
    else
    {
        getPosition(p);
    }

    get2DPosition(pos, p);

    return true;
}

template <typename T>
vector<cv::Point> Vis_transf<T>::expVisibility_obs(cv::Point2i crit, int defl, cv::Mat regions, uchar k, vector<float> extremes, unsigned obt_angle, cv::Mat &vis_map_temp, vector<cv::Point> &vis_map_temp_list)
{
    vector<cv::Point> occ;
    for(int rowx=max((crit.x-defl),0);rowx<=min((crit.x+defl),regions.rows-1);rowx++)
    {
        for(int coly=max((crit.y-defl),0);coly<=min((crit.y+defl),regions.cols-1);coly++)
        {
            float angle=atan2(coly-crit.y,rowx-crit.x);
            float dist=(rowx-crit.x)*(rowx-crit.x)+(coly-crit.y)*(coly-crit.y);

            bool reg;
            if(obt_angle==1)
                reg=(angle<extremes[1] && angle>extremes[0]);
            else
                reg=(angle<extremes[0] || angle>extremes[1]);

            if(reg && (dist<=(1*defl*defl)) && (regions.at<uchar>(rowx,coly)==(k+2) ) )
            {
                vis_map_temp.at<uchar>(rowx,coly)=255;
                vis_map_temp_list.push_back(cv::Point(rowx,coly));
            }
            else if (reg && (dist<=(1*defl*defl)) && (map_or.at<uchar>(rowx,coly)==0) )
            {
                bool stop=false;
                for(int vx=-1;vx<=1;vx++)
                {
                    for(int vy=-1;vy<=1;vy++)
                    {
                        if( (rowx+vx)>=0 && (rowx+vx)<regions.rows && (coly+vy)>=0 && (coly+vy)<regions.cols )
                        {
                            if( (abs(vx)+abs(vy)==1) &&  regions.at<uchar>(rowx+vx,coly+vy)==(k+2)  )
                            {
                                occ.push_back(cv::Point(rowx,coly));
                                stop=true;
                                break;
                            }
                        }
                    }
                    if(stop)
                        break;
                }
            }
        }
    }

    return occ;
}

template <typename T>
vector<cv::Point> Vis_transf<T>::getExtremeFromObstacles(vector<cv::Point> occ, cv::Point2i crit)
{
    vector<vector<cv::Point> > occ_clust=cluster_points(occ);

    vector<cv::Point> occ_critP;

    cv::Mat contours = cv::Mat::ones(map_or.rows, map_or.cols, CV_8UC1)*255;


    for(unsigned int ind=0;ind<occ_clust.size();ind++)
    {
        for(unsigned int occ_p=0;occ_p<occ_clust[ind].size();occ_p++)
        {
            contours.at<uchar>(occ_clust[ind][occ_p].x,occ_clust[ind][occ_p].y)=0;
        }

        if(occ_clust[ind].size()==1)
        {
            occ_critP.push_back(cv::Point(occ_clust[ind][0].x,occ_clust[ind][0].y));
        }
        else
        {
            cv::Mat contours_check=contours.clone();
            bool stop=true;

            while(stop)
            {
                stop=false;
                cv::Point pos;
                for(unsigned int occ_p=0;occ_p<occ_clust[ind].size();occ_p++)
                {
                    if( contours_check.at<uchar>(occ_clust[ind][occ_p].x,occ_clust[ind][occ_p].y)==0  )
                    {
                        pos.x=occ_clust[ind][occ_p].x;
                        pos.y=occ_clust[ind][occ_p].y;
                        stop=true;
                        break;
                    }
                }

                if(stop)
                {
                    BugFollowing bf(contours, contours_check, pos);
                    vector<Chain> chain=bf.getChain();
                    contours_check=bf.getContourChecked();

                    ExtremesObst2Point eo(crit, chain);
                    vector<cv::Point> temp=eo.getExt();
                    occ_critP.insert(occ_critP.end(), temp.begin(), temp.end());
                }
            }

            if(occ_critP.size()==0)
            {
                occ_critP.push_back(cv::Point(occ_clust[ind][0].x,occ_clust[ind][0].y));
            }
        }
    }

    vector<cv::Point> occ_crit_filt;

    cv::Mat contours_filt=contours.clone();

    occ_crit_filt.clear();

    for(unsigned int c=0;c<occ_critP.size();c++)
    {
        if(contours_filt.at<uchar>(occ_critP[c].x,occ_critP[c].y)==255)
            continue;
        else
        {
            contours_filt.at<uchar>(occ_critP[c].x,occ_critP[c].y)=255;
            occ_crit_filt.push_back(cv::Point(occ_critP[c].x,occ_critP[c].y));
        }
    }

    return occ_crit_filt;
}



template <typename T>
void Vis_transf<T>::transf_pos(void)
{
    gt_c=false;
    pos_rcv=false;

    if(count>0 && resCS)
    {
        ros::Time t01=ros::Time::now();

        cv::Point3i pos;

        if(!getPos(pos))
        {
            clearImgs();
            return;
        }

        pos_rcv=true;

        bool proc=checkProceed();

        if(!valid_pos(pos))  //invalid center position of the robot (touching obstacles or walls)
        {
            unsigned char color[3]={255,0,0};

            map_erosionOpPrintColor=printPoint(map_erosionOp, cv::Point(pos.x,pos.y), color);

            map_label=cv::Mat::zeros(map_erosionOp.rows, map_erosionOp.cols, CV_8UC1);
            map_act=cv::Mat::zeros(map_or.rows, map_or.cols, CV_8UC1);
            map_vis=map_act;
            map_truth=map_act;

            act_dist=cv::Mat_<int>::zeros(map_or.rows, map_or.cols);

            bool print=true;

            if (prev.x>=0 && prev.y>=0 && prev.x<map_erosionOp.rows && prev.y<map_erosionOp.cols)
                if (!valid_pos(prev) && !proc)
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

        if(gt)
        {
            gt_c=true;
        }

        visibility(pos, proc, t01);

        prev=pos;
    }
    else if(count>0)
    {
        clearImgs();
        return;
    }
}




template <typename T>
void Vis_transf<T>::publish(void)
{
    if(count>0)
    {
        nav_msgs::OccupancyGrid n_msg;

        n_msg=Mat2RosMsg(map_erosionOpSmall , msg_rcv_pub);
        pub.publish(n_msg);

        n_msg=Mat2RosMsg(map_closeOp , msg_rcv_pub);
        pub2.publish(n_msg);

        if(pos_rcv)
        {
            n_msg=Mat2RosMsg( map_labelSmall , msg_rcv_pub);
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

            std_msgs::Int16MultiArray act_msg;
            act_msg.layout.data_offset=0;
            act_msg.layout.dim.clear();
            std_msgs::MultiArrayDimension dim;
            dim.label="x";
            dim.size=act_dist.rows;
            dim.stride=act_dist.rows*act_dist.cols;
            act_msg.layout.dim.push_back(dim);
            dim.label="y";
            dim.size=act_dist.cols;
            dim.stride=act_dist.cols;
            act_msg.layout.dim.push_back(dim);
            act_msg.data.resize(act_msg.layout.data_offset+act_msg.layout.dim[0].stride);
            for(int i=0;i<act_dist.rows;i++){
                for(int j=0;j<act_dist.cols;j++){
                    act_msg.data[act_msg.layout.data_offset+act_msg.layout.dim[1].stride*i+j]=act_dist(i,j);
                }
            }
            act_dist_pub.publish(act_msg);
        }
    }
}

template <typename T>
void Vis_transf<T>::run(bool _opt)
{
    update(_opt);

    transf();

    transf_pos();

    show();

    if(!pub_once)
        publish();
}

template class Vis_transf<map_transform::ParametersConfig>;
template class Vis_transf<map_transform::ParametersncConfig>;

