#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <map_transform/ParametersConfig.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <thread>
#include <mutex>

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

std::vector<std::vector<cv::Point> >  label(const cv::Mat binary, int conn);
std::vector<cv::Point> label_seed(const cv::Mat binary, int conn, cv::Point seed);

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


    nav_msgs::OccupancyGrid Mat2RosMsg(cv::Mat map ,const nav_msgs::OccupancyGrid& msg);

    void rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    void callbackParameters(map_transform::ParametersConfig &config, uint32_t level);

    bool getTFPosition(cv::Point2d&p);

    bool getPosition(cv::Point2i&pos);

    bool checkProceed(void);

    bool checkProceed2(void);

    bool reachability_map(std::vector<std::vector<cv::Point> > labels, cv::Point2i pos, cv::Mat & r_map);

    vector<cv::Point> expVisibility_obs(cv::Point2i crit, int defl, cv::Mat regions, uchar k, vector<float> extremes, unsigned obt_angle, cv::Mat &vis_map_temp);

    vector<cv::Point> getExtremeFromObstacles(vector<cv::Point> occ, cv::Point2i crit);

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

public:

    Vis_transf(ros::NodeHandle nh): nh_(nh)
    {

        pub = nh_.advertise<nav_msgs::OccupancyGrid>("e_map", 1,true);
        pub2 = nh_.advertise<nav_msgs::OccupancyGrid>("c_map", 1,true);
        pub3 = nh_.advertise<nav_msgs::OccupancyGrid>("r_map", 1,true);
        pub4 = nh_.advertise<nav_msgs::OccupancyGrid>("a_map", 1,true);
        pub5 = nh_.advertise<nav_msgs::OccupancyGrid>("v_map", 1,true);
        pub6 = nh_.advertise<nav_msgs::OccupancyGrid>("g_map", 1,true);

        sub = nh_.subscribe("map", 1, &Vis_transf::rcv_map, this);


        nh_.param("infl", infl, 5);
        nh_.param("defl", defl, infl);

        nh_.param("x", rxr, 10.0);
        nh_.param("y", ryr, 10.0);

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


        if(_debug){
            cv::namedWindow(M_WINDOW);
            cv::namedWindow(E_WINDOW);
            cv::namedWindow(C_WINDOW);
            if(pos_rcv)
            {
                cv::namedWindow(L_WINDOW);
                cv::namedWindow(A_WINDOW);
                cv::namedWindow(D_WINDOW);
                cv::namedWindow(V_WINDOW);
                if(gt && gt_c)
                    cv::namedWindow(G_WINDOW);
            }
        }
    }

    ~Vis_transf()
    {
        if(_debug){
           cv::destroyWindow(M_WINDOW);
           cv::destroyWindow(E_WINDOW);
           cv::destroyWindow(C_WINDOW);
           cv::destroyWindow(L_WINDOW);
           cv::destroyWindow(A_WINDOW);
           cv::destroyWindow(D_WINDOW);
           cv::destroyWindow(V_WINDOW);
           cv::destroyWindow(G_WINDOW);

        }
    }


    void update(void);
    void show(void);
    void publish(void);
    void transf(void);
    void transf_pos(void);
};


void Vis_transf::callbackParameters(map_transform::ParametersConfig &config, uint32_t level) {
    mtx.lock();

    changed_p=true;

    _config=config;

    mtx.unlock();
}


void Vis_transf::show(void)
{
    if(count>0 && _debug){

        cv::imshow(M_WINDOW,map_or);
        cv::imshow(E_WINDOW,map_erosionOpPrintColor);
        cv::imshow(C_WINDOW,map_closeOp);

        if(pos_rcv)
        {
            cv::imshow(L_WINDOW,map_label);
            cv::imshow(A_WINDOW,map_act);
            cv::imshow(V_WINDOW,map_vis);
            cv::imshow(D_WINDOW,map_debug);
            if(gt && gt_c)
            {
                cv::imshow(G_WINDOW,map_truth);
            }
            else
            {
                cv::destroyWindow(G_WINDOW);
                cv::waitKey(2);
            }
        }

        cv::waitKey(3);
    }

    if(!_debug)
    {
       cv::waitKey(2);
       cv::destroyWindow(M_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(E_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(C_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(L_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(A_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(D_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(V_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(G_WINDOW);
       cv::waitKey(2);
    }
}


nav_msgs::OccupancyGrid Vis_transf::Mat2RosMsg(cv::Mat map ,const nav_msgs::OccupancyGrid& msg)
{
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

void Vis_transf::rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("I heard map: [%d]", msg->header.seq);

    cv::Mat prev_map=cv_map.clone();

    cv_map = cv::Mat(msg->info.width, msg->info.height, CV_8UC1);

    treated=true;

    if(count>0)
    {
        if(cv_map.rows!=prev_map.rows || cv_map.cols!=prev_map.cols || res!=msg->info.resolution || or_x!=msg->info.origin.position.x || or_y!=msg->info.origin.position.y )
            treated=false;
    }
    else
        treated=false;


    res= msg->info.resolution;
    height= msg->info.height;
    width= msg->info.width;

    or_x= msg->info.origin.position.x;
    or_y= msg->info.origin.position.y;

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

    treated2=treated;

}

bool Vis_transf::checkProceed2(void)
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


void Vis_transf::transf(void)
{
    bool proc=checkProceed2();

    if(count>0 && (proc) )
    {

        ros::Time t01=ros::Time::now();

        cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                               cv::Size( 2*infl + 1, 2*infl+1 ),
                                               cv::Point( infl, infl ) );

        cv::Mat element_d = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                               cv::Size( 2*defl + 1, 2*defl+1 ),
                                               cv::Point( defl, defl ) );

        cv::Mat or_map, er_map, cr_map;

        or_map=cv_map.clone();
        msg_rcv_pub=msg_rcv;


        cv::erode( or_map, er_map, element);
        cv::dilate( er_map, cr_map, element_d);

        map_or=or_map;
        map_erosionOp=er_map;
        map_closeOp=cr_map;

        ros::Duration diff = ros::Time::now() - t01;

        ROS_INFO("%s - Time for configuration space: %f", tf_pref.c_str(), diff.toSec());
    }

}

class Cluster {
public:
    vector<cv::Point> frontier;
    vector<cv::Point> rest;

    Cluster()
    {
        frontier.clear();
        rest.clear();
    }

    void append(Cluster b)
    {
        frontier.insert(frontier.end(),b.frontier.begin(),b.frontier.end());
        rest.insert(rest.end(),b.rest.begin(),b.rest.end());
    }

    void print(void)
    {
        for (unsigned int i=0;i<frontier.size();i++)
        {
            cout<<frontier[i].x<<" "<<frontier[i].y<<"; ";
        }
        cout<<endl;

        for (unsigned int i=0;i<rest.size();i++)
        {
            cout<<rest[i].x<<" "<<rest[i].y<<"; ";
        }
        cout<<endl;
    }
};


Cluster clustering(Cluster clust, unsigned int index)
{

    clust.frontier.clear();

    if(clust.rest.size()==0 || index>=clust.rest.size())
    {
        return clust;
    }


    Cluster temp=clust;
    temp.rest.erase(temp.rest.begin()+index);
    temp.frontier.push_back(clust.rest[index]);

    Cluster tempL;

    bool done=false;

    while(!done)
    {
        done=true;

        for(unsigned int i=0;i<temp.rest.size();i++)
        {
            if(  (temp.rest[i].x==(clust.rest[index].x+1)|| temp.rest[i].x==clust.rest[index].x || temp.rest[i].x==(clust.rest[index].x-1) ) && ( temp.rest[i].y==(clust.rest[index].y+1) || temp.rest[i].y==clust.rest[index].y || temp.rest[i].y==(clust.rest[index].y-1) ) )
            {

                tempL=clustering(temp,i);

                temp.frontier.insert(temp.frontier.end(),tempL.frontier.begin(),tempL.frontier.end());
                temp.rest=tempL.rest;

                done=false;

                break;

            }

        }
    }
    return temp;
}

vector<vector<cv::Point> > cluster_points(vector<cv::Point> frontiers)
{
    Cluster cl, res;
    cl.frontier.clear();
    cl.rest=frontiers;

    vector<vector<cv::Point> > result;
    result.clear();

    while(cl.rest.size()>0)
    {
        res=clustering(cl,0);

        result.push_back(res.frontier);

        cl=res;
        cl.frontier.clear();
    }

    return result;
}

class Chain{
public:
    int x;
    int y;
    int d;

    Chain(int x_,int y_,int d_):x(x_),y(y_),d(d_)
    {
    }

    bool operator==(Chain const & C)
    {
        return (this->x == C.x &&
                this->y == C.y &&
                this->d == C.d);
    }

    bool operator!=(Chain const & C)
    {
        return !(*this == C);
    }
};

bool raytracing(cv::Mat map, cv::Point2i opt, cv::Point2i ref, cv::Point2i dest, float dist_t, bool (*func)(cv::Mat&,cv::Point2i,cv::Point2i))
{
    float angle=atan2(dest.y-opt.y, dest.x-opt.x);

    float cos_ang=cos(angle);
    float sin_ang=sin(angle);

    float sign_x=0; if(cos_ang!=0) sign_x=cos_ang/abs(cos_ang);
    float sign_y=0; if(sin_ang!=0) sign_y=sin_ang/abs(sin_ang);


    int p_x=ref.x;
    int p_y=ref.y;

    float temp, tempx=ref.x, tempy=ref.y, temp_tx, temp_ty;

    if(cos_ang!=0)
    {
        temp_tx=sign_x*0.5/cos_ang;
        if(sin_ang!=0)
        {
            temp_ty=sign_y*0.5/sin_ang;
            if(temp_tx<temp_ty)
                temp=temp_tx;
            else
                temp=temp_ty;
        }
        else
            temp=temp_tx;

    }
    else
        temp=sign_y*0.5/sin_ang;

    int prev_px=p_x;

    int prev_py=p_y;

    if(sign_x==0)
        p_x=p_x;
    else
        p_x=sign_x*((int)round(sign_x*(tempx+temp*cos_ang)+p_x)-p_x);

    if(sign_y==0)
        p_y=p_y;
    else
        p_y=sign_y*((int)round(sign_y*(tempy+temp*sin_ang)+p_y)-p_y);

    float dist=sqrt( (ref.x-opt.x)*(ref.x-opt.x)+(ref.y-opt.y)*(ref.y-opt.y) );


    while( (dist+temp)<=dist_t && p_x>=0 && p_x<map.rows && p_y>=0 && p_y<map.cols )
    {
        dist=dist+temp;

        if(!(*func)(map, cv::Point2i(p_x,p_y), cv::Point2i(prev_px,prev_py)))
            return false;

        tempx=tempx+temp*cos_ang;
        tempy=tempy+temp*sin_ang;

        if(cos_ang!=0)
        {
            temp_tx=(p_x+sign_x*0.5-tempx)/cos_ang;
            if(temp_tx==0)
                temp_tx=sign_x*1/cos_ang;
            if(sin_ang!=0)
            {
                temp_ty=(p_y+sign_y*0.5-tempy)/sin_ang;
                if(temp_ty==0)
                    temp_ty=sign_y*1/sin_ang;

                if(temp_tx<temp_ty)
                    temp=temp_tx;
                else
                    temp=temp_ty;
            }
            else
                temp=temp_tx;
        }
        else
        {
            temp=(p_y+sign_y*0.5-tempy)/sin_ang;
            if(temp==0)
                temp=sign_y*1/sin_ang;
        }



        prev_px=p_x;

        prev_py=p_y;

        if(sign_x==0)
            p_x=p_x;
        else
            p_x=sign_x*((int)round(sign_x*(tempx+temp*cos_ang)+p_x)-p_x);

        if(sign_y==0)
            p_y=p_y;
        else
            p_y=sign_y*((int)round(sign_y*(tempy+temp*sin_ang)+p_y)-p_y);

    }

    return true;
}


bool checkMap(cv::Mat &map, cv::Point2i p, cv::Point2i prev)
{
    if( map.at<uchar>(p.x,p.y)==0 )
        return false;

    if( (abs(prev.x-p.x)+abs(prev.y-p.y))==2 )
    {
        if( map.at<uchar>(prev.x,p.y)==0 )
            return false;

        if( map.at<uchar>(p.x,prev.y)==0 )
            return false;
    }

    return true;
}

bool print2Map(cv::Mat &map, cv::Point2i p, cv::Point2i prev)//, cv::Point2i prev)
{
    map.at<uchar>(p.x,p.y)=0;

    return true;
}


bool raytracing(cv::Mat map, int opt_x, int opt_y, int dest_x, int dest_y)
{
    float dist_t=sqrt( (dest_x-opt_x)*(dest_x-opt_x)+(dest_y-opt_y)*(dest_y-opt_y) );
    return raytracing(map, cv::Point2i(opt_x,opt_y), cv::Point2i(opt_x,opt_y), cv::Point2i(dest_x,dest_y), dist_t, &checkMap);
}


cv::Mat brute_force(cv::Mat map, cv::Mat reach, int defl)
{
    cv::Mat result=map.clone();

    for(int i=0;i<map.rows;i++)
    {
        for(int j=0;j<map.cols;j++)
        {
            if(map.at<uchar>(i,j)==0)
                continue;

            bool stop=false;

            for(int ii=0;ii<reach.rows;ii++)
            {
                for(int jj=0;jj<reach.cols;jj++)
                {
                    if(reach.at<uchar>(ii,jj)==0)
                        continue;

                    if( ( (i-ii)*(i-ii)+(j-jj)*(j-jj) )>defl*defl )
                        continue;

                    if( raytracing(map,ii,jj,i,j) )
                    {
                        stop=true;
                        break;
                    }

                }

                if(stop)
                    break;
            }

            if(stop)
                continue;

            result.at<uchar>(i,j)=0;


        }
    }

    return result;
}


cv::Mat brute_force_opt_act(cv::Mat map, cv::Mat reach, cv::Mat act, int defl)
{

    cv::Mat result=map.clone();

    for(int i=0;i<map.rows;i++)
    {
        for(int j=0;j<map.cols;j++)
        {
            if(map.at<uchar>(i,j)==0)
                continue;

            if(act.at<uchar>(i,j)==255)
            {
                continue;
            }

            bool stop=false;

            for(int r=0; r<=defl; r++)
            {
                if(r==0)
                {
                    if(reach.at<uchar>(i,j)==0)
                        ;
                    else{
                        stop=true;
                        break;

                    }
                }
                else
                {
                    for(int p=-r;p<r;p++)
                    {
                        int ii=i-r, jj=j+p;

                        if( (ii)<map.rows &&  (ii)>=0 && (jj)<map.rows &&  (jj)>=0 )
                        {
                            if(reach.at<uchar>(ii,jj)==0)
                                ;
                            else{
                                if( ( (i-ii)*(i-ii)+(j-jj)*(j-jj) )>defl*defl )
                                    ;
                                else{
                                    if( raytracing(map,ii,jj,i,j) )
                                    {
                                        stop=true;
                                        break;
                                    }
                                }
                            }

                        }

                        ii=i+p, jj=j+r;

                        if( (ii)<map.rows &&  (ii)>=0 && (jj)<map.rows &&  (jj)>=0 )
                        {
                            if(reach.at<uchar>(ii,jj)==0)
                                ;
                            else{
                                if( ( (i-ii)*(i-ii)+(j-jj)*(j-jj) )>defl*defl )
                                    ;
                                else{
                                    if( raytracing(map,ii,jj,i,j) )
                                    {
                                        stop=true;
                                        break;
                                    }
                                }
                            }

                        }

                        ii=i+r, jj=j-p;

                        if( (ii)<map.rows &&  (ii)>=0 && (jj)<map.rows &&  (jj)>=0 )
                        {
                            if(reach.at<uchar>(ii,jj)==0)
                                ;
                            else{
                                if( ( (i-ii)*(i-ii)+(j-jj)*(j-jj) )>defl*defl )
                                    ;
                                else{
                                    if( raytracing(map,ii,jj,i,j) )
                                    {
                                        stop=true;
                                        break;
                                    }
                                }
                            }

                        }

                        ii=i-p, jj=j-r;

                        if( (ii)<map.rows &&  (ii)>=0 && (jj)<map.rows &&  (jj)>=0 )
                        {
                            if(reach.at<uchar>(ii,jj)==0)
                                ;
                            else{
                                if( ( (i-ii)*(i-ii)+(j-jj)*(j-jj) )>defl*defl )
                                    ;
                                else{
                                    if( raytracing(map,ii,jj,i,j) )
                                    {
                                        stop=true;
                                        break;
                                    }
                                }
                            }
                        }
                    }

                }

                if(stop)
                    break;
            }

            if(stop)
                continue;

            result.at<uchar>(i,j)=0;

        }
    }


    return result;
}


cv::Mat brute_force_opt(cv::Mat map, cv::Mat reach, int defl)
{
    cv::Mat result=map.clone();

    for(int i=0;i<map.rows;i++)
    {
        for(int j=0;j<map.cols;j++)
        {
            if(map.at<uchar>(i,j)==0)
                continue;

            bool stop=false;

            for(int r=0; r<=defl; r++)
            {
                if(r==0)
                {
                    if(reach.at<uchar>(i,j)==0)
                        ;
                    else{
                        stop=true;
                        break;

                    }
                }
                else
                {
                    for(int p=-r;p<r;p++)
                    {
                        int ii=i-r, jj=j+p;

                        if( (ii)<map.rows &&  (ii)>=0 && (jj)<map.rows &&  (jj)>=0 )
                        {
                            if(reach.at<uchar>(ii,jj)==0)
                                ;
                            else{
                                if( ( (i-ii)*(i-ii)+(j-jj)*(j-jj) )>defl*defl )
                                    ;
                                else{
                                    if( raytracing(map,ii,jj,i,j) )
                                    {
                                        stop=true;
                                        break;
                                    }
                                }
                            }
                        }

                        ii=i+p, jj=j+r;

                        if( (ii)<map.rows &&  (ii)>=0 && (jj)<map.rows &&  (jj)>=0 )
                        {
                            if(reach.at<uchar>(ii,jj)==0)
                                ;
                            else{
                                if( ( (i-ii)*(i-ii)+(j-jj)*(j-jj) )>defl*defl )
                                    ;
                                else{
                                    if( raytracing(map,ii,jj,i,j) )
                                    {
                                        stop=true;
                                        break;
                                    }
                                }
                            }
                        }

                        ii=i+r, jj=j-p;

                        if( (ii)<map.rows &&  (ii)>=0 && (jj)<map.rows &&  (jj)>=0 )
                        {
                            if(reach.at<uchar>(ii,jj)==0)
                                ;
                            else{
                                if( ( (i-ii)*(i-ii)+(j-jj)*(j-jj) )>defl*defl )
                                    ;
                                else{
                                    if( raytracing(map,ii,jj,i,j) )
                                    {
                                        stop=true;
                                        break;
                                    }
                                }
                            }
                        }

                        ii=i-p, jj=j-r;

                        if( (ii)<map.rows &&  (ii)>=0 && (jj)<map.rows &&  (jj)>=0 )
                        {
                            if(reach.at<uchar>(ii,jj)==0)
                                ;
                            else{
                                if( ( (i-ii)*(i-ii)+(j-jj)*(j-jj) )>defl*defl )
                                    ;
                                else{
                                    if( raytracing(map,ii,jj,i,j) )
                                    {
                                        stop=true;
                                        break;
                                    }
                                }
                            }
                        }
                    }

                }

                if(stop)
                    break;
            }

            if(stop)
                continue;

            result.at<uchar>(i,j)=0;

        }
    }

    return result;
}

bool Vis_transf::getTFPosition(cv::Point2d &p){
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
    return true;
}

int boundPos(int x, int max)
{
    if(x>=max)
        x=max-1;
    if(x<0)
        x=0;
    return x;
}

bool Vis_transf::getPosition(cv::Point2i& pos){
    cv::Point2d p;
    if(!_debug)
    {
        if(!getTFPosition(p))
            return false;
    }
    else
    {
        p.x=rxr;
        p.y=ryr;
    }

    pos.x=(int) round((p.x-or_x)/res);
    pos.y=(int) round((p.y-or_y)/res);


    pos.x=boundPos(pos.x, map_or.rows);
    pos.y=boundPos(pos.y, map_or.cols);

    return true;
}


bool Vis_transf::checkProceed(void)
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

cv::Mat printPoint(cv::Mat img, cv::Point2i pos, unsigned char* color)
{
    vector<cv::Mat> channels(3);

    channels[0]=img.clone();
    channels[1]=img.clone();
    channels[2]=img.clone();

    for(int k=0;k<3;k++)
    {
        for(int i=(pos.x-1);i<=(pos.x+1);i++)
        {
            for(int j=(pos.y-1);j<=(pos.y+1);j++)
            {

                channels[k].at<uchar>(boundPos(i,img.rows),boundPos(j,img.cols))=color[2-k];
            }
        }
    }

    cv::Mat ret;

    cv::merge(channels, ret);

    return ret;
}


bool Vis_transf::reachability_map(std::vector<std::vector<cv::Point> > labels, cv::Point2i pos, cv::Mat & r_map)
{
    bool found_pos=false, found_prev=false;
    unsigned int label_pos=0, prev_label=0;


    for (unsigned int i=0;i<labels.size();i++){
        for (unsigned int j=0;j<labels[i].size();j++){
            if (pos.x==labels[i][j].x && pos.y==labels[i][j].y){
                label_pos=i+1;
                found_pos=true;
            }
            if(prev.x>=0 &&  prev.y>=0)
            {
                if(prev.x==labels[i][j].x && prev.y==labels[i][j].y){
                    prev_label=i+1;
                    found_prev=true;
                }
            }
            if ( found_pos && ((found_prev) || (prev.x < 0) || (prev.y < 0) ))
                break;
        }
        if(found_pos && ( (found_prev) || (prev.x<0) || (prev.y<0) ) )
            break;
    }

    for (unsigned int i=0;i<labels.size();i++){
        if( i!=(label_pos-1) ){
            for(unsigned int j=0;j<labels[i].size();j++){
                r_map.at<uchar>(labels[i][j].x,labels[i][j].y)=0;
            }
        }
     }

    return (prev_label!=label_pos) && found_pos; //returns true if reachable set changes from prev position
}


class Unreachable
{
    vector<vector<cv::Point> > labels_unreach;
    cv::Mat act;
    void getRegions(cv::Mat map_or, cv::Mat act_map);
public:
    cv::Mat regions;
    vector<vector<vector<cv::Point> > > frontiers;
    cv::Mat unreach_map;

    Unreachable(cv::Mat map_or, cv::Mat act_map) {
        getRegions(map_or, act_map);
    }
    void getFrontiers(void);

};

void Unreachable::getFrontiers(void)
{
    frontiers.clear();
    for (unsigned int k=0;k<labels_unreach.size();k++){
        vector<cv::Point> frontiers_t;
        for(unsigned int j=0;j<labels_unreach[k].size();j++){
            bool stop=false;
            for(int a=labels_unreach[k][j].x-1;a<=(labels_unreach[k][j].x+1);a++){
                for(int b=labels_unreach[k][j].y-1;b<=(labels_unreach[k][j].y+1);b++){
                    if ((a>=0)&&(b>=0)&&(a<act.rows)&&(b<act.cols) )
                    {
                        int aa=a-labels_unreach[k][j].x;
                        int bb=b-labels_unreach[k][j].y;
                        if((abs(aa)+abs(bb))==1)   ///TODO: check connectivity, if 4 is enough!!!
                            if(act.at<uchar>(a,b)==255)
                            {
                                frontiers_t.push_back(cv::Point(labels_unreach[k][j].x,labels_unreach[k][j].y));
                                stop=true;
                                break;
                            }
                    }
                }
                if(stop)
                    break;
            }
        }
        vector<vector<cv::Point> > frontiers_c=cluster_points(frontiers_t);
        frontiers.push_back(frontiers_c);
    }
}

void Unreachable::getRegions(cv::Mat map_or, cv::Mat act_map)
{
    cv::Mat temp_labelling, temp;

    bitwise_not(map_or.clone(),temp);

    unreach_map=act_map|temp;

    bitwise_not( unreach_map.clone() , temp_labelling);

    labels_unreach=label(temp_labelling/255,4);

    regions=map_or.clone()/255;

    for (unsigned int i=0;i<labels_unreach.size();i++){
        for(unsigned int j=0;j<labels_unreach[i].size();j++){
               regions.at<uchar>(labels_unreach[i][j].x,labels_unreach[i][j].y)=i+2;
        }
    }

    act=act_map;
}


template <typename T, typename T2=T>
class FindElem
{
protected:
    int n;
    int ind;
    T fv;
    T2 p;


    virtual bool func(T var)=0;
public:
    void iter(T var) {
        if(n==0)
        {
            ind=0;
            fv=var;
        }
        else if(func(var))
        {
            ind=n;
            fv=var;
        }
        n++;
    }
    void iter(T var,T2 pt) {
        if(n==0)
        {
            ind=0;
            fv=var;
            p=pt;
        }
        else if(func(var))
        {
            ind=n;
            fv=var;
            p=pt;
        }
        n++;
    }

    FindElem()
    {
        n=0;
        ind=0;
        fv=T();
        p=T2();
    }

    void clear(void)
    {
        n=0;
        ind=0;
        fv=T();
        p=T2();
    }

    int getInd(void)
    {
        return ind;
    }
    T getVal(void)
    {
        return fv;
    }
    T2 getP(void)
    {
        return p;
    }
};

template <typename T, typename T2=T>
class FindMax : public FindElem<T,T2>
{
protected:
    bool func(T var)
    {
        if(var>FindElem<T,T2>::fv)
            return true;
        else
            return false;
    }
public:
    FindMax(vector<T> vars)
    {
        for(int i=0;i<vars.size();i++)
        {
            this->iter(vars[i]);
        }
    }
    FindMax(vector<T> vars, vector<T2> pts)
    {
        for(int i=0;i<vars.size();i++)
        {
            this->iter(vars[i],pts[i]);
        }
    }
    FindMax(void)
    {
    }
};

template <typename T,typename T2=T>
class FindMin : public FindElem<T,T2>
{
protected:
    bool func(T var)
    {
        if(var<this->fv)
            return true;
        else
            return false;
    }
public:
    FindMin(vector<T> vars)
    {
        for(int i=0;i<vars.size();i++)
        {
            this->iter(vars[i]);
        }
    }
    FindMin(vector<T> vars, vector<T2> pts)
    {
        for(int i=0;i<vars.size();i++)
        {
            this->iter(vars[i],pts[i]);
        }
    }
    FindMin(void)
    {
    }
};



template <typename T, typename T2=T>
class OrderedVector
{
protected:
    vector<T> fv;
    vector<T2> p;

    virtual bool func(T var, T comp)=0;
public:
    unsigned int iter(T var) {
        typename vector<T>::iterator it=fv.begin();
        for(unsigned int i=0; i<fv.size(); i++)
        {
            if(func(var,fv[i]))
            {
                fv.insert(it,var);
                return i;
            }
            it++;
        }
        fv.insert(fv.end(),var);
        return fv.size()-1;
    }
    unsigned int iter(T var,T2 pt) {
        typename vector<T>::iterator it=fv.begin();
        typename vector<T2>::iterator itp=p.begin();
        for(unsigned int i=0; i<fv.size(); i++)
        {
            if(func(var,fv[i]))
            {
                fv.insert(it,var);
                p.insert(itp,pt);
                return i;
            }
            it++;
            itp++;
        }
        fv.insert(fv.end(),var);
        p.insert(p.end(),pt);
        return fv.size()-1;
    }

    OrderedVector()
    {
        fv.clear();
        p.clear();
    }

    void clear(void)
    {
        fv.clear();
        p.clear();
    }

    unsigned getSize(void)
    {
        return fv.size();
    }
    T getVal(unsigned int ind)
    {
        if(ind<fv.size())
            return fv[ind];
        else
            return T();
    }
    T2 getP(unsigned int ind)
    {
        if(ind<p.size())
            return p[ind];
        else
            return T2();
    }
};

template <typename T, typename T2=T>
class OrderedDecresc : public OrderedVector<T,T2>
{
protected:
    bool func(T var, T comp)
    {
        if(var>comp)
            return true;
        else
            return false;
    }
public:
    OrderedDecresc(void)
    {
    }
};

template <typename T, typename T2=T>
class OrderedCresc : public OrderedVector<T,T2>
{
protected:
    bool func(T var, T comp)
    {
        if(var<comp)
            return true;
        else
            return false;
    }
public:
    OrderedCresc(void)
    {
    }
};


class Find_Obtuse_Angle
{
private:
    unsigned int obt_angle;
public:
    OrderedCresc<float, cv::Point2i> angles;

    Find_Obtuse_Angle(void)
    {
        obt_angle=0;
        angles=OrderedCresc<float,cv::Point2i>();
    }

    void initialize(float a0, float a1)
    {
        if((a1-a0)>PI)
            obt_angle=0;
        else
            obt_angle=1;
    }

    void updateOA(int i0, int i1, int i2, bool p1, bool p2)
    {
        float a1,a2;
        if(p1)
            a1=2*PI;
        else
            a1=0;
        if(p2)
            a2=2*PI;
        else
            a2=0;

        if( (angles.getVal(i1)-angles.getVal(i0)+a1)>PI  )
        {
            obt_angle=i0;
        }
        else if( (angles.getVal(i2)-angles.getVal(i1)+a2)>PI  )
        {
            obt_angle=i1;
        }
        else
        {
            FindMax<float> anglediff;
            for(unsigned int aa=0;aa<(angles.getSize()-1);aa++)
            {
                anglediff.iter(angles.getVal(aa+1)-angles.getVal(aa));
            }
            anglediff.iter( (angles.getVal(0)+PI)+(PI-angles.getVal(angles.getSize()-1)) );

            obt_angle=anglediff.getInd();
        }
    }

    void iter(float angle, cv::Point2i pt)
    {
        unsigned int a=angles.iter(angle,pt);

        if(angles.getSize()==2)
        {
            initialize(angles.getVal(0),angles.getVal(1));
        }
        else if(angles.getSize()>2)
        {
            if(a==(angles.getSize()-1))
            {
                if( (a)==(obt_angle+1) )
                {
                    updateOA(obt_angle, a, 0, false, true);
                }
            }
            else
            {
                if(a==(obt_angle+1) )
                {
                    updateOA(obt_angle, a, a+1, false, false);
                }
                else if( a==0 && (obt_angle+1)==(angles.getSize()-1) )
                {
                    updateOA(obt_angle+1, a, a+1, true, false);
                }
                else if(a<=obt_angle)
                {
                    obt_angle+=1;
                }
            }
        }
    }

    unsigned int getObt(void)
    {
        return obt_angle;
    }

    void clear(void)
    {
        obt_angle=0;
        angles=OrderedCresc<float,cv::Point2i>();
    }
};


class CritPoints
{
private:
    cv::Mat r_map, map_or;
    int infl;
    cv::Point2i critP;
    vector<cv::Point2i> frontier;
    vector<float> extremes;
    vector<cv::Point2i> extremesP;
    unsigned int obt;
    cv::Point2i find_extreme(cv::Point2i pt, float a0, float a1, bool special=false);
    void extremePoints(cv::Point2i pt, cv::Point2i pt2, float a0, float a1, bool first=false);
    float getAngle(cv::Point2i pt);
public:
    CritPoints(cv::Mat map, cv::Mat reach, int rs): r_map(reach), map_or(map), infl(rs)
    {
    }
    cv::Point2i find_crit_point(vector<cv::Point> frontier);
    vector<float> frontier_extremes(void);
    cv::Point2i getCrit(void)
    {
        return critP;
    }
    vector<float> getExtremes(void)
    {
        return extremes;
    }
    vector<cv::Point2i> getExtremesP(void)
    {
        return extremesP;
    }
    unsigned int getObt(void)
    {
        return obt;
    }
};

cv::Point2i CritPoints::find_crit_point(vector<cv::Point> frontier_p)
{
    FindMin<int> min_y, min_x;
    FindMax<int> max_y, max_x;

    for(unsigned int j=0;j<frontier_p.size();j++){
        max_x.iter(frontier_p[j].x);
        min_x.iter(frontier_p[j].x);
        max_y.iter(frontier_p[j].y);
        min_y.iter(frontier_p[j].y);
    }

    FindMin<double, cv::Point2i> crit;

    for(int x=max(min_x.getVal()-infl,0);x<min(max_x.getVal()+infl,r_map.rows);x++)
    {
        for(int y=max(min_y.getVal()-infl,0);y<min(max_y.getVal()+infl,r_map.cols);y++)
        {
            if(r_map.at<uchar>(x,y)==255)
            {
                double sum=0;
                for(unsigned int l=0;l<frontier_p.size();l++){
                    sum+=(frontier_p[l].x-x)*(frontier_p[l].x-x)+(frontier_p[l].y-y)*(frontier_p[l].y-y);
                }
                crit.iter(sum,cv::Point2i(x,y));
            }
        }
    }

    critP=crit.getP();
    frontier=frontier_p;

    return critP;
}


cv::Point2i CritPoints::find_extreme(cv::Point2i pt, float a0, float a1, bool special)
{
    FindMin<float,cv::Point2i> mp;
    for(int rowx=max((pt.x-1),0);rowx<=min((pt.x+1),map_or.rows-1);rowx++)
    {
        for(int coly=max((pt.y-1),0);coly<=min((pt.y+1),map_or.cols-1);coly++)
        {
            float angle=atan2(coly-critP.y,rowx-critP.x);
            bool inside_region;

            if(special)
                inside_region=(
                                ( (a0>a1) && (angle<a1) && ( (a0-angle)<PI ) ) ||
                                ( (a0<angle) && (angle<a1) && ( (a0+2*PI-angle)<PI ) ) ||
                                ( (a0>a1) && (angle>a0) && ( (a0+2*PI-angle)<PI ) ) ||
                                ( (angle>a1) && (a0<a1) && ( (angle-a0)<PI ) ) ||
                                ( (angle<a0) && (a0<a1) && ( (angle+2*PI-a0)<PI ) ) ||
                                ( (angle>a1) && (a0>angle) && ( (angle+2*PI-a0)<PI ) )
                              );
            else
            {
                if(a0<a1)
                    inside_region=(angle>a0 && angle<a1);
                else
                    inside_region=(angle>a0 || angle<a1);
            }

            if(inside_region  && map_or.at<uchar>(rowx,coly)==0)
            {
                float dist=(rowx-critP.x)*(rowx-critP.x)+(coly-critP.y)*(coly-critP.y);
                mp.iter(dist,cv::Point2i(rowx,coly));
            }
        }
    }

    return mp.getP();


}

float CritPoints::getAngle(cv::Point2i pt){
    return atan2(pt.y-critP.y,pt.x-critP.x);
}

void CritPoints::extremePoints(cv::Point2i pt, cv::Point2i pt2, float a0, float a1, bool first)
{
    extremes.clear();
    extremesP.clear();
    float e;
    cv::Point2i ptf;

    if(first)
        ptf=find_extreme(pt, -4*PI, 4*PI);
    else
        ptf=find_extreme(pt, a0, a1);

    extremesP.push_back(ptf);
    e=getAngle(ptf);
    extremes.push_back(e);

    if(first)
        ptf=find_extreme(pt2, extremes[0], a1, true);
    else
        ptf=find_extreme(pt2, a0, a1);

    e=getAngle(ptf);

    if( e<extremes[0] )
    {
        extremes.insert(extremes.begin(),e);
        extremesP.insert(extremesP.begin(),ptf);
    }
    else
    {
        extremes.push_back(e);
        extremesP.push_back(ptf);
    }

    if( (extremes[1]-extremes[0])>PI )
        obt=0;
    else
        obt=1;
}

vector<float> CritPoints::frontier_extremes(void)
{
    Find_Obtuse_Angle oa;

    for(unsigned int l=0;l<frontier.size();l++)
    {
        float angle=atan2(frontier[l].y-critP.y,frontier[l].x-critP.x);

        oa.iter(angle,cv::Point2i(frontier[l].x,frontier[l].y));
    }

    if(oa.angles.getSize()==1)
    {
        extremePoints(oa.angles.getP(0), oa.angles.getP(0), 0, oa.angles.getVal(0), true);
    }
    else if(oa.angles.getSize()>1)
    {
        if(oa.getObt()==(oa.angles.getSize()-1))
        {
            extremePoints(oa.angles.getP(oa.getObt()), oa.angles.getP(0), oa.angles.getVal(oa.getObt()), oa.angles.getVal(0));
        }
        else
        {
            extremePoints(oa.angles.getP(oa.getObt()), oa.angles.getP(oa.getObt()+1), oa.angles.getVal(oa.getObt()), oa.angles.getVal(oa.getObt()+1));
        }
    }

    return extremes;
}


vector<cv::Point> Vis_transf::expVisibility_obs(cv::Point2i crit, int defl, cv::Mat regions, uchar k, vector<float> extremes, unsigned obt_angle, cv::Mat &vis_map_temp)
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


class ExtremesObst2Point
{
private:
    cv::Point2i critP;
    vector<Chain> ch;
    vector<float> diffAngle;
    float prev_diff;
    vector<cv::Point> ext;
    void chainAngleDiff(void);
    void firstLoop(void);
    void secLoop(void);
public:
    ExtremesObst2Point(cv::Point2i crit, vector<Chain> c): critP(crit), ch(c)
    {
        chainAngleDiff();
        ext.clear();
        prev_diff=0;
        firstLoop();
        secLoop();
    }
    vector<cv::Point2i> getExt(void)
    {
        return ext;
    }

};

void ExtremesObst2Point::chainAngleDiff(void)
{
    diffAngle.clear();
    float prev_angle, act_angle;
    for(unsigned int c=0;c<ch.size();c++)
    {
        act_angle=atan2(ch[c].y-critP.y,ch[c].x-critP.x);
        if(c>0)
        {
            float diff_angle=act_angle-prev_angle;

            if(diff_angle<-PI)
                diff_angle+=2*PI;
            else if (diff_angle>PI)
                diff_angle-=2*PI;

            diffAngle.push_back(diff_angle);
        }
        prev_angle=act_angle;
    }
}


void ExtremesObst2Point::firstLoop(void)
{
    for(unsigned int c=0;c<diffAngle.size();c++)
    {
        if(c>0)
        {
            if( diffAngle[c]*prev_diff<0)
            {
                ext.push_back(cv::Point(ch[c].x,ch[c].y));
                prev_diff=diffAngle[c];
            }
        }
        else
            prev_diff=diffAngle[c];
    }
}

void ExtremesObst2Point::secLoop(void)
{
    for(unsigned int c=0;c<diffAngle.size();c++)
    {
        if( diffAngle[c]*prev_diff<0)
        {
            ext.push_back(cv::Point(ch[c].x,ch[c].y));
            break;
        }
        else if(diffAngle[c]*prev_diff>0)
        {
            break;
        }
    }
}


class BugFollowing
{
private:
    const static int dir=8; // number of possible directions to go at any position
    //if (dir==4){
    //    static int dx[dir]={1, 0, -1, 0};
    //    static int dy[dir]={0, 1, 0, -1};
    //}
    //if (dir==8) {
    const int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};
    //}
    int prev_d;
    int sign;
    cv::Mat contours, contours_check;
    cv::Point pos;
    vector<Chain> chain;
    void run(void);
    bool iter(void);
    void initialization(void);
    void saveP(int d);
    void save(int d);
    bool check_direction(int sn, int d, int max_dir);
public:
    BugFollowing(cv::Mat con, cv::Mat con_ch, cv::Point ini): contours(con), contours_check(con_ch), pos(ini)
    {
        chain.clear();
        prev_d=0;
        sign=1;
        run();
    }
    vector<Chain> getChain(void)
    {
        return chain;
    }
    cv::Mat getContourChecked(void)
    {
        return contours_check;
    }
};

void BugFollowing::saveP(int d)
{
    contours_check.at<uchar>(pos.x,pos.y)=255;
    pos.x=pos.x+dx[d];
    pos.y=pos.y+dy[d];
    prev_d=(d+dir/2)%dir;
}

void BugFollowing::save(int d)
{
    Chain c(pos.x,pos.y,d);
    chain.push_back(c);
    saveP(d);
}

bool BugFollowing::iter(void)
{
    int act_dir;
    for(int d=1;d<=dir;d++)
    {
        act_dir=(prev_d+sign*d+dir)%dir;
        if( (pos.x+dx[act_dir])>=0 && (pos.x+dx[act_dir])<contours.rows && (pos.y+dy[act_dir])>=0 && (pos.y+dy[act_dir])<contours.cols )
        {
            if( contours.at<uchar>(pos.x+dx[act_dir],pos.y+dy[act_dir])==0)
            {
                Chain c(pos.x,pos.y,act_dir);
                saveP(act_dir);

                if(chain[chain.size()-1]==c)
                {
                    return true;
                }

                chain.push_back(c);

                if(chain[0]==c)
                {
                    return true;
                }
                break;
            }
        }
    }
    return false;
}

bool BugFollowing::check_direction(int sn, int d, int max_dir)
{
    bool valid_rotation=true;
    for(int dd=1;dd<=max_dir;dd++)
    {
        int act_dir=(prev_d+sn*dd+dir)%dir;
        if( (pos.x+dx[d]+dx[act_dir])>=0 && (pos.x+dx[d]+dx[act_dir])<contours.rows && (pos.y+dy[d]+dy[act_dir])>=0 && (pos.y+dy[d]+dy[act_dir])<contours.cols )
        {
            if( contours.at<uchar>(pos.x+dx[d]+dx[act_dir],pos.y+dy[d]+dy[act_dir])==0)
            {
                valid_rotation=false;
                break;
            }
            else
            {
                valid_rotation=true;
            }
        }
    }
    return valid_rotation;
}

void BugFollowing::initialization(void)
{
    for(int d=0;d<dir;d++)
    {
        if( (pos.x+dx[d])>=0 && (pos.x+dx[d])<contours.rows && (pos.y+dy[d])>=0 && (pos.y+dy[d])<contours.cols )
        {
            if(  contours.at<uchar>(pos.x+dx[d],pos.y+dy[d])==0 )
            {
                prev_d=(d+dir/2)%dir;
                int max_dir;

                if(prev_d==0 || prev_d==2 || prev_d==4 || prev_d==6)
                    max_dir=2;
                else
                    max_dir=1;

                if(check_direction(1,d,max_dir))
                {
                    sign=1;
                    save(d);
                    break;
                }
                else
                {
                    if(check_direction(-1,d,max_dir))
                    {
                        sign=-1;
                        save(d);
                        break;
                    }
                    else
                        continue;
                }
            }
        }
    }
}

void BugFollowing::run(void)
{
    bool cont_cond=true;

    while(cont_cond)
    {
        if(chain.size()!=0)
        {
            cont_cond=!iter();
        }
        else
        {
            initialization();
        }
    }
}


vector<cv::Point> Vis_transf::getExtremeFromObstacles(vector<cv::Point> occ, cv::Point2i crit)
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

void Vis_transf::transf_pos(void)
{
    if(count>0)
    {
        ros::Time t01=ros::Time::now();

        cv::Point2i pos;

        if (!getPosition(pos))
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


        std::vector<std::vector<cv::Point> > labels=label(map_erosionOp.clone()/255,8);

        cv::Mat r_map=map_erosionOp.clone();

        bool new_v=reachability_map(labels,pos,r_map);


        if( (new_v) || (prev.x<0) || (prev.y<0) || proc )  //if visibility is changed
        {
            int rad=min(infl,defl);  //if defl<infl, visibility is given by morphological closing


            cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                                   cv::Size( 2*rad + 1, 2*rad+1 ),
                                                   cv::Point( rad, rad ) );

            cv::Mat act_map=r_map.clone();

            dilate( act_map, act_map, element);  //actuation space

            cv::Mat vis_map=act_map.clone();

            Unreachable unreach(map_or, act_map);

            if(infl<defl)  //extended sensing radius
            {
                unreach.getFrontiers();

                cv::Mat regions=unreach.regions;

                cv::Mat vis_map_temp;

                CritPoints critP(map_or, r_map, infl);

                for (unsigned int k=0;k<unreach.frontiers.size();k++){//2;k++){//
                    for(unsigned int ff=0;ff<unreach.frontiers[k].size();ff++)
                    {
                        vector<cv::Point> frontier=unreach.frontiers[k][ff];

                        if(frontier.size()>0)
                        {
                            cv::Point2i crit=critP.find_crit_point(frontier);

                            critP.frontier_extremes();

                            //// TODO:neighbor points

                            vis_map_temp = cv::Mat::zeros(regions.rows, regions.cols, CV_8UC1)*255;

                            vector<cv::Point> occ=expVisibility_obs(crit, defl, regions, k, critP.getExtremes(), critP.getObt(), vis_map_temp);

                            vector<cv::Point> occ_crit_filt=getExtremeFromObstacles(occ, crit);

                            for(unsigned int c=0;c<occ_crit_filt.size();c++)
                            {
                                raytracing(vis_map_temp, cv::Point2i(crit.x,crit.y), occ_crit_filt[c], occ_crit_filt[c], defl, &print2Map);
                            }

                            for(unsigned int j=0;j<frontier.size();j++){
                                if(vis_map_temp.at<uchar>( frontier[j].x,frontier[j].y)==255)
                                {
                                    std::vector<cv::Point> points_vis=label_seed(vis_map_temp.clone()/255,4,cv::Point(frontier[j].x,frontier[j].y));
                                    for(unsigned int pv=0;pv<points_vis.size();pv++)
                                    {
                                        vis_map.at<uchar>(points_vis[pv].x,points_vis[pv].y)=255;
                                    }
                                    break;
                                }
                            }
                        }
                    }
                }
            }

            map_label=r_map;
            map_act=act_map;
            map_vis=vis_map;

            map_debug=unreach.unreach_map;

            unsigned char color[3]={0,255,0};

            map_erosionOpPrintColor=printPoint(map_erosionOp, pos, color);

            ros::Duration diff = ros::Time::now() - t01;

            ROS_INFO("%s - Time for visibility: %f", tf_pref.c_str(), diff.toSec());

            if(gt)
            {
                ros::Time t3=ros::Time::now();

                map_truth=brute_force_opt(map_or, map_label, defl);

                diff = ros::Time::now() - t3;

                ROS_INFO("%s - Time for  Optimized brute force: %f", tf_pref.c_str(), diff.toSec());

                gt_c=true;
            }
        }


        pos_rcv=true;

        prev=pos;
    }
    else
        return;
}

void Vis_transf::update(void)
{
    bool proc=false;

    map_transform::ParametersConfig config;

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
        changed=true;
        changed2=true;

        infl=config.infl;
        defl=config.defl;
        _debug=config.debug;
        gt=config.ground_truth;
        rxr=config.x;
        ryr=config.y;

    }

}



void Vis_transf::publish(void)
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


std::vector<std::vector<cv::Point> >  label(const cv::Mat binary, int conn)
{
    std::vector<std::vector<cv::Point> > blobs;
    blobs.clear();

    cv::Mat label_image;
    binary.convertTo(label_image, CV_32FC1);

    int label_count = 2; // starts at 2 because 0,1 are used already

    for(int y=0; y < binary.rows; y++) {
        for(int x=0; x < binary.cols; x++) {
            if((int)label_image.at<float>(y,x) != 1 ) {
                continue;
            }

            cv::Rect rect;
            cv::floodFill(label_image, cv::Point(x,y), cv::Scalar(label_count), &rect, cv::Scalar(0), cv::Scalar(0), conn);

            std::vector<cv::Point>  blob;

            for(int i=rect.y; i < (rect.y+rect.height); i++) {
                for(int j=rect.x; j < (rect.x+rect.width); j++) {
                    if((int)label_image.at<float>(i,j) != label_count) {
                        continue;
                    }

                    blob.push_back(cv::Point(i,j));
                }
            }

            blobs.push_back(blob);

            label_count++;
        }
    }

    return blobs;
}

std::vector<cv::Point> label_seed(const cv::Mat binary, int conn, cv::Point seed)
{
    std::vector<cv::Point> blob;
    blob.clear();

    cv::Mat label_image;
    binary.convertTo(label_image, CV_32FC1);

    int label_count = 2; // starts at 2 because 0,1 are used already

    if((int)label_image.at<float>(seed.x,seed.y) != 1 ) {
                return blob;
    }

    cv::Rect rect;
    cv::Point sd=cv::Point(seed.y, seed.x);
    cv::floodFill(label_image, sd, cv::Scalar(label_count), &rect, cv::Scalar(0), cv::Scalar(0), conn);


    for(int i=rect.y; i < (rect.y+rect.height); i++) {
        for(int j=rect.x; j < (rect.x+rect.width); j++) {
            if((int)label_image.at<float>(i,j) != label_count) {
                continue;
            }

            blob.push_back(cv::Point(i,j));
        }
    }

    return blob;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "visibility");

    ros::NodeHandle nh("~");

    Vis_transf vis(nh);

    ros::Rate loop_rate(10);


    while (ros::ok())
    {
        vis.update();

        ros::spinOnce();

        vis.transf();

        vis.transf_pos();

        vis.show();

        vis.publish();

        loop_rate.sleep();
    }


    return 0;
}
