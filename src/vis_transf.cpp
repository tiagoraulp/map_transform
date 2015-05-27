#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <map_transform/ParametersConfig.h>

#include <sstream>
#include <string>

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

    std::vector<std::vector<cv::Point> >  label(const cv::Mat binary, int conn);
    std::vector<cv::Point> label_seed(const cv::Mat binary, int conn, cv::Point seed);
    nav_msgs::OccupancyGrid Mat2RosMsg(cv::Mat map ,const nav_msgs::OccupancyGrid& msg);

    void rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    void callbackParameters(map_transform::ParametersConfig &config, uint32_t level);

    int count;

    bool pos_rcv;

    bool treated, treated2;

    int infl;

    int defl;

    int prev_x, prev_y;

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
        pub3 = nh_.advertise<nav_msgs::OccupancyGrid>("vis_map", 1,true);
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


        prev_x=-1; prev_y=-1;

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

    void show(void);
    void publish(void);
    void transf(void);

    void update(void);


    void transf_pos(void);


    bool getTreated(void){return treated;}


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


void Vis_transf::transf(void)
{
    bool proc=false;


    if((!treated2) || changed2)
    {
        proc=true;
        treated2=true;
        changed2=false;
    }


    if(count>0 && (proc) )
    {

        ros::Time t01=ros::Time::now();

        cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                               cv::Size( 2*infl + 1, 2*infl+1 ),
                                               cv::Point( infl, infl ) );

        cv::Mat element_d = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                               cv::Size( 2*defl + 1, 2*defl+1 ),
                                               cv::Point( defl, defl ) );

        cv::Mat or_map, er_map, cl_map;

        or_map=cv_map.clone();
        msg_rcv_pub=msg_rcv;


        cv::erode( or_map, er_map, element);
        cv::dilate( er_map, cl_map, element_d);

        map_or=or_map;
        map_erosionOp=er_map;
        map_closeOp=cl_map;

        ros::Duration diff = ros::Time::now() - t01;

        ROS_INFO("%s - Time for reach: %f", tf_pref.c_str(), diff.toSec());
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
        for (int i=0;i<frontier.size();i++)
        {
            cout<<frontier[i].x<<" "<<frontier[i].y<<"; ";
        }
        cout<<endl;

        for (int i=0;i<rest.size();i++)
        {
            cout<<rest[i].x<<" "<<rest[i].y<<"; ";
        }
        cout<<endl;
    }
};


Cluster clustering(Cluster clust, int index)
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

        for(int i=0;i<temp.rest.size();i++)
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

bool raytracing(cv::Mat map, int opt_x, int opt_y, int dest_x, int dest_y)
{
    float dist_t=sqrt( (dest_x-opt_x)*(dest_x-opt_x)+(dest_y-opt_y)*(dest_y-opt_y) );
    float angle=atan2(dest_y-opt_y, dest_x-opt_x);

    float cos_ang=cos(angle);
    float sin_ang=sin(angle);

    float sign_x=0; if(cos_ang!=0) sign_x=cos_ang/abs(cos_ang);
    float sign_y=0; if(sin_ang!=0) sign_y=sin_ang/abs(sin_ang);


    int p_x=opt_x;
    int p_y=opt_y;

    float temp, tempx=opt_x, tempy=opt_y, temp_tx, temp_ty;

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

    float dist=0;


    while( (dist+temp)<=dist_t && p_x>=0 && p_x<map.rows && p_y>=0 && p_y<map.cols )
    {
        dist=dist+temp;


        if( map.at<uchar>(p_x,p_y)==0 )
            return false;

        if( (abs(prev_px-p_x)+abs(prev_py-p_y))==2 )
        {
            if( map.at<uchar>(prev_px,p_y)==0 )
                return false;

            if( map.at<uchar>(p_x,prev_py)==0 )
                return false;
        }

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


void Vis_transf::transf_pos(void)
{



    if(count>0)
    {
        ros::Time t01=ros::Time::now();

        double xx,yy;

        if(!_debug)
        {
            tf::StampedTransform transform;
            try{
                pos_listener.lookupTransform("/map", tf_pref+"/base_link", ros::Time(0), transform);
            }
            catch (tf::TransformException ex){
              ROS_INFO("%s",ex.what());
              return ;
            }
            xx=transform.getOrigin().x();
            yy=transform.getOrigin().y();
        }
        else
        {
            xx=rxr;
            yy=ryr;
        }


        int pos_x=(int) round((xx-or_x)/res);

        int pos_y=(int) round((yy-or_y)/res);

        bool proc=false;

        if((!treated) || changed)
        {
            proc=true;
            treated=true;
            changed=false;
        }

        if(map_erosionOp.at<uchar>(pos_x,pos_y)==0  && (proc) )
        {
            pos_rcv=false;

            prev_x=pos_x;
            prev_y=pos_y;

            gt_c=false;

            vector<cv::Mat> channels(3);

            channels[0]=map_erosionOp.clone();
            channels[1]=map_erosionOp.clone();
            channels[2]=map_erosionOp.clone();

            channels[2].at<uchar>(pos_x-1,pos_y-1)=255;
            channels[2].at<uchar>(pos_x-1,pos_y)=255;
            channels[2].at<uchar>(pos_x-1,pos_y+1)=255;
            channels[2].at<uchar>(pos_x,pos_y-1)=255;
            channels[2].at<uchar>(pos_x,pos_y)=255;
            channels[2].at<uchar>(pos_x,pos_y+1)=255;
            channels[2].at<uchar>(pos_x+1,pos_y-1)=255;
            channels[2].at<uchar>(pos_x+1,pos_y)=255;
            channels[2].at<uchar>(pos_x+1,pos_y+1)=255;

            channels[1].at<uchar>(pos_x-1,pos_y-1)=0;
            channels[1].at<uchar>(pos_x-1,pos_y)=0;
            channels[1].at<uchar>(pos_x-1,pos_y+1)=0;
            channels[1].at<uchar>(pos_x,pos_y-1)=0;
            channels[1].at<uchar>(pos_x,pos_y)=0;
            channels[1].at<uchar>(pos_x,pos_y+1)=0;
            channels[1].at<uchar>(pos_x+1,pos_y-1)=0;
            channels[1].at<uchar>(pos_x+1,pos_y)=0;
            channels[1].at<uchar>(pos_x+1,pos_y+1)=0;

            channels[0].at<uchar>(pos_x-1,pos_y-1)=0;
            channels[0].at<uchar>(pos_x-1,pos_y)=0;
            channels[0].at<uchar>(pos_x-1,pos_y+1)=0;
            channels[0].at<uchar>(pos_x,pos_y-1)=0;
            channels[0].at<uchar>(pos_x,pos_y)=0;
            channels[0].at<uchar>(pos_x,pos_y+1)=0;
            channels[0].at<uchar>(pos_x+1,pos_y-1)=0;
            channels[0].at<uchar>(pos_x+1,pos_y)=0;
            channels[0].at<uchar>(pos_x+1,pos_y+1)=0;


            cv::merge(channels, map_erosionOpPrintColor);

            map_label=cv::Mat::zeros(map_erosionOp.rows, map_erosionOp.cols, CV_8UC1);
            map_act=cv::Mat::zeros(map_or.rows, map_or.cols, CV_8UC1);
            map_vis=cv::Mat::zeros(map_or.rows, map_or.cols, CV_8UC1);

            ros::Duration diff = ros::Time::now() - t01;

            ROS_INFO("%s - Time for label: %f", tf_pref.c_str(), diff.toSec());

            return;


        }

        cv::Mat temp_labelling, temp;

        std::vector<std::vector<cv::Point> > labels=label(map_erosionOp.clone()/255,8);


        int label_pos=-1, prev_label=-1;


        for (int i=0;i<labels.size();i++){
            for(int j=0;j<labels[i].size();j++){
                if( pos_x==labels[i][j].x && pos_y==labels[i][j].y){
                    label_pos=i;
                }
                if( prev_x==labels[i][j].x && prev_y==labels[i][j].y){
                    prev_label=i;
                }
                if(label_pos>-1 && prev_label>-1)
                    break;
            }
            if(label_pos>-1 && prev_label>-1)
                break;
        }

        if( (prev_label!=label_pos) || (prev_x<0) || (prev_y<0) || proc )
        {
            cv::Mat l_map=map_erosionOp.clone(), unreach_map, regions;

            for (int i=0;i<labels.size();i++){
                for(int j=0;j<labels[i].size();j++){
                    if( i!=label_pos ){
                        l_map.at<uchar>(labels[i][j].x,labels[i][j].y)=0;
                    }
                }
             }


            int rad=min(infl,defl);


            cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                                   cv::Size( 2*rad + 1, 2*rad+1 ),
                                                   cv::Point( rad, rad ) );

            cv::Mat act_map=l_map.clone();


            dilate( act_map, act_map, element);


            cv::Mat vis_map=act_map.clone(), vis_map_temp, contours;

            bitwise_not(map_or,temp);

            unreach_map=vis_map|temp;

            bitwise_not( unreach_map.clone() , temp_labelling);

            std::vector<std::vector<cv::Point> > labels_unreach=label(temp_labelling/255,4);

            regions=map_or.clone()/255;

            for (int i=0;i<labels_unreach.size();i++){
                for(int j=0;j<labels_unreach[i].size();j++){
                        regions.at<uchar>(labels_unreach[i][j].x,labels_unreach[i][j].y)=i+2;
                }
            }

            if(infl<defl)
            {
                for (int k=0;k<labels_unreach.size();k++){//2;k++){//
                    vector<cv::Point> frontiers;

                    for(int j=0;j<labels_unreach[k].size();j++){

                        if ((labels_unreach[k][j].x+1)<vis_map.rows)
                            if(vis_map.at<uchar>(labels_unreach[k][j].x+1,labels_unreach[k][j].y)==255)
                            {
                                frontiers.push_back(cv::Point(labels_unreach[k][j].x,labels_unreach[k][j].y));
                                continue;
                            }
                        if ((labels_unreach[k][j].y+1)<vis_map.cols)
                            if(vis_map.at<uchar>(labels_unreach[k][j].x,labels_unreach[k][j].y+1)==255)
                            {
                                frontiers.push_back(cv::Point(labels_unreach[k][j].x,labels_unreach[k][j].y));
                                continue;
                            }
                        if ((labels_unreach[k][j].x-1)>=0)
                            if(vis_map.at<uchar>(labels_unreach[k][j].x-1,labels_unreach[k][j].y)==255)
                            {
                                frontiers.push_back(cv::Point(labels_unreach[k][j].x,labels_unreach[k][j].y));
                                continue;
                            }
                        if ((labels_unreach[k][j].y-1)>=0)
                            if(vis_map.at<uchar>(labels_unreach[k][j].x,labels_unreach[k][j].y-1)==255)
                            {
                                frontiers.push_back(cv::Point(labels_unreach[k][j].x,labels_unreach[k][j].y));
                                continue;
                            }
                    }

                    vector<vector<cv::Point> > frontier=cluster_points(frontiers);


                    for(int ff=0;ff<frontier.size();ff++)
                    {
                        if(frontier[ff].size()>0)
                        {
                            int min_x=vis_map.rows, min_y=vis_map.cols, max_x=-1, max_y=-1;
                            for(int j=0;j<frontier[ff].size();j++){

                                if(frontier[ff][j].x>max_x)
                                    max_x=frontier[ff][j].x;
                                if(frontier[ff][j].y>max_y)
                                    max_y=frontier[ff][j].y;
                                if(frontier[ff][j].x<min_x)
                                    min_x=frontier[ff][j].x;
                                if(frontier[ff][j].y<min_y)
                                    min_y=frontier[ff][j].y;
                            }

                            double min_sum=-1; int opt_x, opt_y;

                            for(int x=max(min_x-infl,0);x<min(max_x+infl,vis_map.rows);x++)
                            {
                                for(int y=max(min_y-infl,0);y<min(max_y+infl,vis_map.cols);y++)
                                {
                                    if(l_map.at<uchar>(x,y)==255)
                                    {
                                        double sum=0;
                                        for(int l=0;l<frontier[ff].size();l++){
                                            sum+=(frontier[ff][l].x-x)*(frontier[ff][l].x-x)+(frontier[ff][l].y-y)*(frontier[ff][l].y-y);
                                        }
                                        if(min_sum==-1)
                                        {
                                              min_sum=sum;
                                              opt_x=x;
                                              opt_y=y;
                                        }
                                        else
                                        {
                                            if(sum<min_sum)
                                            {
                                                min_sum=sum;
                                                opt_x=x;
                                                opt_y=y;
                                            }
                                        }
                                    }


                                }
                            }

                            vector<float> angles;
                            vector<int> angles_x;
                            vector<int> angles_y;

                            int obt_angle=-1;

                            for(int l=0;l<frontier[ff].size();l++)
                            {
                                float angle=atan2(frontier[ff][l].y-opt_y,frontier[ff][l].x-opt_x);
                                vector<float>::iterator it=angles.begin();
                                vector<int>::iterator itx=angles_x.begin();
                                vector<int>::iterator ity=angles_y.begin();
                                if(angles.size()==0)
                                {
                                    angles.push_back(angle);
                                    angles_x.push_back(frontier[ff][l].x);
                                    angles_y.push_back(frontier[ff][l].y);
                                }
                                else
                                {
                                    for(int a=0;a<angles.size();a++)
                                    {
                                        if(angle<angles[a])
                                        {
                                            angles.insert(it,angle);
                                            angles_x.insert(itx,frontier[ff][l].x);
                                            angles_y.insert(ity,frontier[ff][l].y);

                                            if(angles.size()==2)
                                            {
                                                if((angles[1]-angles[0])>PI)
                                                    obt_angle=0;
                                                else
                                                    obt_angle=1;
                                            }
                                            else if(angles.size()>2)
                                            {
                                                if(a==(obt_angle+1) )
                                                {
                                                    if( (angles[a]-angles[obt_angle])>PI  )
                                                    {
                                                        obt_angle=obt_angle;
                                                    }
                                                    else if(  (angles[a+1]-angles[a])>PI  )
                                                    {
                                                        obt_angle=a;
                                                    }
                                                    else
                                                    {
                                                        bool found=false;
                                                        float anglediff;

                                                        for(int aa=0;aa<(angles.size()-1);aa++)
                                                        {
                                                            if(!found)
                                                            {
                                                                anglediff=angles[aa+1]-angles[aa];
                                                                obt_angle=0;
                                                                found=true;
                                                            }
                                                            else
                                                            {
                                                                if( (angles[aa+1]-angles[aa])>anglediff )
                                                                {
                                                                    anglediff=angles[aa+1]-angles[aa];
                                                                    obt_angle=aa;
                                                                }
                                                            }
                                                        }
                                                        if( ((angles[0]+PI)+(PI-angles[angles.size()-1]))>anglediff )
                                                        {
                                                            obt_angle=angles.size()-1;
                                                        }
                                                    }
                                                }
                                                else if( a==0 && (obt_angle+1)==(angles.size()-1) )
                                                {
                                                    if( ( (angles[a]+PI)+(PI-angles[obt_angle+1]) )>PI  )
                                                    {
                                                        obt_angle=obt_angle+1;
                                                    }
                                                    else if(  (angles[a+1]-angles[a])>PI  )
                                                    {
                                                        obt_angle=a;
                                                    }
                                                    else
                                                    {
                                                        bool found=false;
                                                        float anglediff;

                                                        for(int aa=0;aa<(angles.size()-1);aa++)
                                                        {
                                                            if(!found)
                                                            {
                                                                anglediff=angles[aa+1]-angles[aa];
                                                                obt_angle=0;
                                                                found=true;
                                                            }
                                                            else
                                                            {
                                                                if( (angles[aa+1]-angles[aa])>anglediff )
                                                                {
                                                                    anglediff=angles[aa+1]-angles[aa];
                                                                    obt_angle=aa;
                                                                }
                                                            }
                                                        }
                                                        if( ((angles[0]+PI)+(PI-angles[angles.size()-1]))>anglediff )
                                                        {
                                                            obt_angle=angles.size()-1;
                                                        }
                                                    }
                                                }
                                                else
                                                {
                                                     if(a<=obt_angle)
                                                         obt_angle+=1;
                                                }
                                            }
                                            break;
                                        }
                                        it++;
                                        itx++;
                                        ity++;
                                    }
                                    if(angle>=angles[angles.size()-1])
                                    {
                                        angles.insert(angles.end(),angle);
                                        angles_x.insert(angles_x.end(),frontier[ff][l].x);
                                        angles_y.insert(angles_y.end(),frontier[ff][l].y);

                                        if(angles.size()==2)
                                        {
                                            if((angles[1]-angles[0])>PI)
                                                obt_angle=0;
                                            else
                                                obt_angle=1;

                                        }
                                        else if(angles.size()>2)
                                        {
                                            if( (angles.size()-1)==(obt_angle+1) )
                                            {
                                                if( (angles[(angles.size()-1)]-angles[obt_angle])>PI  )
                                                {
                                                    obt_angle=obt_angle;
                                                }
                                                else if(  ( (angles[0]+PI)+(PI-angles[(angles.size()-1)]) )>PI  )
                                                {
                                                    obt_angle=(angles.size()-1);
                                                }
                                                else
                                                {
                                                    bool found=false;

                                                    float anglediff;

                                                    for(int aa=0;aa<(angles.size()-1);aa++)
                                                    {
                                                        if(!found)
                                                        {
                                                            anglediff=angles[aa+1]-angles[aa];
                                                            obt_angle=0;
                                                            found=true;
                                                        }
                                                        else
                                                        {
                                                            if( (angles[aa+1]-angles[aa])>anglediff )
                                                            {
                                                                anglediff=angles[aa+1]-angles[aa];
                                                                obt_angle=aa;
                                                            }
                                                        }
                                                    }
                                                    if( ((angles[0]+PI)+(PI-angles[angles.size()-1]))>anglediff )
                                                    {
                                                        obt_angle=angles.size()-1;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            vector<float> extremes;extremes.clear();

                            if(angles.size()==1)
                            {
                                float min_dist=-1;
                                int min_x, min_y;

                                for(int rowx=max((angles_x[0]-1),0);rowx<=min((angles_x[0]+1),regions.rows-1);rowx++)
                                {
                                    for(int coly=max((angles_y[0]-1),0);coly<=min((angles_y[0]+1),regions.cols-1);coly++)
                                    {
                                        if( map_or.at<uchar>(rowx,coly)==0)
                                        {
                                            float dist=(rowx-opt_x)*(rowx-opt_x)+(coly-opt_y)*(coly-opt_y);
                                            if(min_dist==-1)
                                            {
                                                min_dist=dist;
                                                min_x=rowx;
                                                min_y=coly;
                                            }
                                            else
                                            {
                                                if(dist<min_dist)
                                                {
                                                    min_dist=dist;
                                                    min_x=rowx;
                                                    min_y=coly;
                                                }
                                            }
                                        }
                                    }
                                }
                                extremes.push_back(atan2(min_y-opt_y,min_x-opt_x));

                                min_dist=-1;
                                for(int rowx=max((angles_x[0]-1),0);rowx<=min((angles_x[0]+1),regions.rows-1);rowx++)
                                {
                                    for(int coly=max((angles_y[0]-1),0);coly<=min((angles_y[0]+1),regions.cols-1);coly++)
                                    {
                                        float angle=atan2(coly-opt_y,rowx-opt_x);
                                        if(map_or.at<uchar>(rowx,coly)==0)
                                        {

                                            if (
                                                 ( (extremes[0]>angles[0]) && (angle<angles[0]) && ( (extremes[0]-angle)<PI ) ) ||
                                                 ( (extremes[0]<angle) && (angle<angles[0]) && ( (extremes[0]+2*PI-angle)<PI ) ) ||
                                                 ( (extremes[0]>angles[0]) && (angle>extremes[0]) && ( (extremes[0]+2*PI-angle)<PI ) ) ||
                                                 ( (angle>angles[0]) && (extremes[0]<angles[0]) && ( (angle-extremes[0])<PI ) ) ||
                                                 ( (angle<extremes[0]) && (extremes[0]<angles[0]) && ( (angle+2*PI-extremes[0])<PI ) ) ||
                                                 ( (angle>angles[0]) && (extremes[0]>angle) && ( (angle+2*PI-extremes[0])<PI ) )
                                               )
                                            {
                                                float dist=(rowx-opt_x)*(rowx-opt_x)+(coly-opt_y)*(coly-opt_y);
                                                if(min_dist==-1)
                                                {
                                                    min_dist=dist;
                                                    min_x=rowx;
                                                    min_y=coly;
                                                }
                                                else
                                                {
                                                    if(dist<min_dist)
                                                    {
                                                        min_dist=dist;
                                                        min_x=rowx;
                                                        min_y=coly;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }

                                if( atan2(min_y-opt_y,min_x-opt_x)<extremes[0] )
                                    extremes.insert(extremes.begin(),atan2(min_y-opt_y,min_x-opt_x));
                                else
                                    extremes.push_back(atan2(min_y-opt_y,min_x-opt_x));

                                if( (extremes[1]-extremes[0])>PI )
                                    obt_angle=0;
                                else
                                    obt_angle=1;

                            }
                            else if(angles.size()>1)
                            {
                                if(obt_angle==(angles.size()-1))
                                {
                                    float min_dist=-1;
                                    int min_x, min_y;
                                    for(int rowx=max((angles_x[obt_angle]-1),0);rowx<=min((angles_x[obt_angle]+1),regions.rows-1);rowx++)
                                    {
                                        for(int coly=max((angles_y[obt_angle]-1),0);coly<=min((angles_y[obt_angle]+1),regions.cols-1);coly++)
                                        {
                                            float angle=atan2(coly-opt_y,rowx-opt_x);
                                            if( (angle>angles[obt_angle] || angle<angles[0] ) && map_or.at<uchar>(rowx,coly)==0)
                                            {
                                                float dist=(rowx-opt_x)*(rowx-opt_x)+(coly-opt_y)*(coly-opt_y);
                                                if(min_dist==-1)
                                                {
                                                    min_dist=dist;
                                                    min_x=rowx;
                                                    min_y=coly;
                                                }
                                                else
                                                {
                                                    if(dist<min_dist)
                                                    {
                                                        min_dist=dist;
                                                        min_x=rowx;
                                                        min_y=coly;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    extremes.push_back(atan2(min_y-opt_y,min_x-opt_x));

                                    min_dist=-1;
                                    for(int rowx=max((angles_x[0]-1),0);rowx<=min((angles_x[0]+1),regions.rows-1);rowx++)
                                    {
                                        for(int coly=max((angles_y[0]-1),0);coly<=min((angles_y[0]+1),regions.cols-1);coly++)
                                        {
                                            float angle=atan2(coly-opt_y,rowx-opt_x);
                                            if((angle>angles[obt_angle] || angle<angles[0]) && map_or.at<uchar>(rowx,coly)==0)
                                            {
                                                float dist=(rowx-opt_x)*(rowx-opt_x)+(coly-opt_y)*(coly-opt_y);
                                                if(min_dist==-1)
                                                {
                                                    min_dist=dist;
                                                    min_x=rowx;
                                                    min_y=coly;
                                                }
                                                else
                                                {
                                                    if(dist<min_dist)
                                                    {
                                                        min_dist=dist;
                                                        min_x=rowx;
                                                        min_y=coly;
                                                    }
                                                }
                                            }
                                        }
                                    }

                                    if( atan2(min_y-opt_y,min_x-opt_x)<extremes[0] )
                                        extremes.insert(extremes.begin(),atan2(min_y-opt_y,min_x-opt_x));
                                    else
                                        extremes.push_back(atan2(min_y-opt_y,min_x-opt_x));

                                    if( (extremes[1]-extremes[0])>PI )
                                        obt_angle=0;
                                    else
                                        obt_angle=1;
                                }
                                else
                                {
                                    float min_dist=-1;
                                    int min_x, min_y;
                                    for(int rowx=max((angles_x[obt_angle]-1),0);rowx<=min((angles_x[obt_angle]+1),regions.rows-1);rowx++)
                                    {
                                        for(int coly=max((angles_y[obt_angle]-1),0);coly<=min((angles_y[obt_angle]+1),regions.cols-1);coly++)
                                        {
                                            float angle=atan2(coly-opt_y,rowx-opt_x);
                                            if(angle>angles[obt_angle] && angle<angles[obt_angle+1] && map_or.at<uchar>(rowx,coly)==0)
                                            {
                                                float dist=(rowx-opt_x)*(rowx-opt_x)+(coly-opt_y)*(coly-opt_y);
                                                if(min_dist==-1)
                                                {
                                                    min_dist=dist;
                                                    min_x=rowx;
                                                    min_y=coly;
                                                }
                                                else
                                                {
                                                    if(dist<min_dist)
                                                    {
                                                        min_dist=dist;
                                                        min_x=rowx;
                                                        min_y=coly;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    extremes.push_back(atan2(min_y-opt_y,min_x-opt_x));

                                    min_dist=-1;
                                    for(int rowx=max((angles_x[obt_angle+1]-1),0);rowx<=min((angles_x[obt_angle+1]+1),regions.rows-1);rowx++)
                                    {
                                        for(int coly=max((angles_y[obt_angle+1]-1),0);coly<=min((angles_y[obt_angle+1]+1),regions.cols-1);coly++)
                                        {
                                            float angle=atan2(coly-opt_y,rowx-opt_x);
                                            if(angle>angles[obt_angle] && angle<angles[obt_angle+1] && map_or.at<uchar>(rowx,coly)==0)
                                            {
                                                float dist=(rowx-opt_x)*(rowx-opt_x)+(coly-opt_y)*(coly-opt_y);
                                                if(min_dist==-1)
                                                {
                                                    min_dist=dist;
                                                    min_x=rowx;
                                                    min_y=coly;
                                                }
                                                else
                                                {
                                                    if(dist<min_dist)
                                                    {
                                                        min_dist=dist;
                                                        min_x=rowx;
                                                        min_y=coly;
                                                    }
                                                }
                                            }
                                        }
                                    }

                                    if( atan2(min_y-opt_y,min_x-opt_x)<extremes[0] )
                                        extremes.insert(extremes.begin(),atan2(min_y-opt_y,min_x-opt_x));
                                    else
                                        extremes.push_back(atan2(min_y-opt_y,min_x-opt_x));

                                    if( (extremes[1]-extremes[0])>PI )
                                        obt_angle=0;
                                    else
                                        obt_angle=1;
                                }
                            }

                            //// TODO:neighbor points

                            vis_map_temp = cv::Mat::zeros(regions.rows, regions.cols, CV_8UC1)*255;

                            vector<cv::Point> pre_vis; pre_vis.clear();

                            vector<cv::Point> occ;

                            for(int rowx=max((opt_x-defl),0);rowx<=min((opt_x+defl),regions.rows-1);rowx++)
                            {
                                for(int coly=max((opt_y-defl),0);coly<=min((opt_y+defl),regions.cols-1);coly++)
                                {
                                    float angle=atan2(coly-opt_y,rowx-opt_x);
                                    float dist=(rowx-opt_x)*(rowx-opt_x)+(coly-opt_y)*(coly-opt_y);

                                    if(obt_angle==1)
                                    {
                                        if(angle<extremes[1] && angle>extremes[0] && (dist<=(1*defl*defl)) && (regions.at<uchar>(rowx,coly)==(k+2) ) )
                                        {
                                            vis_map_temp.at<uchar>(rowx,coly)=255;
                                            pre_vis.push_back(cv::Point(rowx,coly));
                                        }
                                        else if (angle<extremes[1] && angle>extremes[0] && (dist<=(1*defl*defl)) && (map_or.at<uchar>(rowx,coly)==0) )
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
                                    else
                                    {
                                        if( (angle<extremes[0] || angle>extremes[1]) && (dist<=(1*defl*defl)) && (regions.at<uchar>(rowx,coly)==(k+2) ) )
                                        {
                                            vis_map_temp.at<uchar>(rowx,coly)=255;
                                            pre_vis.push_back(cv::Point(rowx,coly));
                                        }
                                        else if ( (angle<extremes[0] || angle>extremes[1]) && (dist<=(1*defl*defl)) && (map_or.at<uchar>(rowx,coly)==0) )
                                        {
                                            bool stop=false;
                                            for(int vx=-1;vx<=1;vx++)
                                            {
                                                for(int vy=-1;vy<=1;vy++)
                                                {
                                                    if( (rowx+vx)>=0 && (rowx+vx)<regions.rows && (coly+vy)>=0 && (coly+vy)<regions.cols )
                                                    {
                                                        if( (abs(vx)+abs(vy)==1) && regions.at<uchar>(rowx+vx,coly+vy)==(k+2)  )
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
                            }

                            vector<vector<cv::Point> > occ_clust=cluster_points(occ);

                            vector<cv::Point> occ_critP;

                            contours = cv::Mat::ones(regions.rows, regions.cols, CV_8UC1)*255;

                            for(int ind=0;ind<occ_clust.size();ind++)
                            {
                                for(int occ_p=0;occ_p<occ_clust[ind].size();occ_p++)
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
                                        int pos_x,pos_y;
                                        for(int occ_p=0;occ_p<occ_clust[ind].size();occ_p++)
                                        {
                                            if( contours_check.at<uchar>(occ_clust[ind][occ_p].x,occ_clust[ind][occ_p].y)==0  )
                                            {
                                                pos_x=occ_clust[ind][occ_p].x;
                                                pos_y=occ_clust[ind][occ_p].y;
                                                stop=true;
                                                break;
                                            }                                                  
                                        }

                                        if(stop)
                                        {
                                            const int dir=8; // number of possible directions to go at any position
                                            //if (dir==4){
                                            //    static int dx[dir]={1, 0, -1, 0};
                                            //    static int dy[dir]={0, 1, 0, -1};
                                            //}
                                            //if (dir==8) {
                                                static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
                                                static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};
                                            //}

                                            vector<Chain> chain;
                                            chain.clear();

                                            int prev_d;
                                            bool cont_cond=true;
                                            int sign=1;

                                            while(cont_cond)
                                            {
                                                if(chain.size()!=0)
                                                {
                                                    int act_dir;
                                                    for(int d=1;d<=dir;d++)
                                                    {
                                                        act_dir=(prev_d+sign*d+dir)%dir;
                                                        if( (pos_x+dx[act_dir])>=0 && (pos_x+dx[act_dir])<regions.rows && (pos_y+dy[act_dir])>=0 && (pos_y+dy[act_dir])<regions.cols )
                                                        {
                                                            if( contours.at<uchar>(pos_x+dx[act_dir],pos_y+dy[act_dir])==0)
                                                            {
                                                                Chain c(pos_x,pos_y,act_dir);
                                                                contours_check.at<uchar>(pos_x,pos_y)=255;

                                                                if(chain[chain.size()-1]==c)
                                                                {
                                                                    cont_cond=false;
                                                                    break;
                                                                }

                                                                chain.push_back(c);

                                                                prev_d=(act_dir+dir/2)%dir;
                                                                pos_x=pos_x+dx[act_dir];
                                                                pos_y=pos_y+dy[act_dir];

                                                                if(chain[0]==c)
                                                                {
                                                                    cont_cond=false;
                                                                }
                                                                break;
                                                            }
                                                        }
                                                    }

                                                }
                                                else
                                                {
                                                    bool valid_rotation;
                                                    for(int d=0;d<dir;d++)
                                                    {
                                                        if( (pos_x+dx[d])>=0 && (pos_x+dx[d])<regions.rows && (pos_y+dy[d])>=0 && (pos_y+dy[d])<regions.cols )
                                                        {
                                                            if(  contours.at<uchar>(pos_x+dx[d],pos_y+dy[d])==0 )
                                                            {
                                                                prev_d=(d+dir/2)%dir;
                                                                int max_dir;

                                                                if(prev_d==0 || prev_d==2 || prev_d==4 || prev_d==6)
                                                                    max_dir=2;
                                                                else
                                                                    max_dir=1;

                                                                int act_dir;

                                                                valid_rotation=true;
                                                                sign=1;                                                                
                                                                for(int dd=1;dd<=max_dir;dd++)
                                                                {
                                                                    act_dir=(prev_d+sign*dd)%dir;
                                                                    if( (pos_x+dx[d]+dx[act_dir])>=0 && (pos_x+dx[d]+dx[act_dir])<regions.rows && (pos_y+dy[d]+dy[act_dir])>=0 && (pos_y+dy[d]+dy[act_dir])<regions.cols )
                                                                    {
                                                                        if( contours.at<uchar>(pos_x+dx[d]+dx[act_dir],pos_y+dy[d]+dy[act_dir])==0)
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

                                                                if(valid_rotation)
                                                                {
                                                                    sign=1;
                                                                    Chain c(pos_x,pos_y,d);
                                                                    contours_check.at<uchar>(pos_x,pos_y)=255;
                                                                    pos_x=pos_x+dx[d];
                                                                    pos_y=pos_y+dy[d];
                                                                    chain.push_back(c);
                                                                    break;
                                                                }
                                                                else
                                                                {
                                                                    valid_rotation=true;
                                                                    sign=-1;
                                                                    for(int dd=1;dd<=max_dir;dd++)
                                                                    {
                                                                        act_dir=(prev_d+sign*dd+dir)%dir;
                                                                        if( (pos_x+dx[d]+dx[act_dir])>=0 && (pos_x+dx[d]+dx[act_dir])<regions.rows && (pos_y+dy[d]+dy[act_dir])>=0 && (pos_y+dy[d]+dy[act_dir])<regions.cols )
                                                                        {
                                                                            if( contours.at<uchar>(pos_x+dx[d]+dx[act_dir],pos_y+dy[d]+dy[act_dir])==0)
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

                                                                    if(valid_rotation)
                                                                    {
                                                                        sign=-1;
                                                                        Chain c(pos_x,pos_y,d);
                                                                        contours_check.at<uchar>(pos_x,pos_y)=255;
                                                                        pos_x=pos_x+dx[d];
                                                                        pos_y=pos_y+dy[d];
                                                                        chain.push_back(c);
                                                                        break;
                                                                    }
                                                                    else
                                                                    {
                                                                        continue;
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }

                                            float prev_angle, act_angle;

                                            vector<float> diffs;
                                            diffs.clear();

                                            for(int c=0;c<chain.size();c++)
                                            {
                                                act_angle=atan2(chain[c].y-opt_y,chain[c].x-opt_x);
                                                if(c>0)
                                                {
                                                    float diff_angle=act_angle-prev_angle;

                                                    if(diff_angle<-PI)
                                                        diff_angle+=2*PI;
                                                    else if (diff_angle>PI)
                                                        diff_angle-=2*PI;

                                                    diffs.push_back(diff_angle);
                                                }
                                                prev_angle=act_angle;
                                            }


                                            float prev_diff;
                                            for(int c=0;c<diffs.size();c++)
                                            {

                                                if(c>0)
                                                {
                                                    if( diffs[c]*prev_diff<0)
                                                    {
                                                        occ_critP.push_back(cv::Point(chain[c].x,chain[c].y));
                                                        prev_diff=diffs[c];
                                                    }
                                                }
                                                else
                                                    prev_diff=diffs[c];
                                            }

                                            for(int c=0;c<diffs.size();c++)
                                            {
                                                if( diffs[c]*prev_diff<0)
                                                {
                                                    occ_critP.push_back(cv::Point(chain[c].x,chain[c].y));
                                                    prev_diff=diffs[c];
                                                    break;
                                                }
                                                else if(diffs[c]*prev_diff>0)
                                                {
                                                    break;
                                                }
                                            }
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

                            for(int c=0;c<occ_critP.size();c++)
                            {
                                if(contours_filt.at<uchar>(occ_critP[c].x,occ_critP[c].y)==255)
                                    continue;
                                else
                                {
                                    contours_filt.at<uchar>(occ_critP[c].x,occ_critP[c].y)=255;
                                    occ_crit_filt.push_back(cv::Point(occ_critP[c].x,occ_critP[c].y));
                                }
                            }

                            for(int c=0;c<occ_crit_filt.size();c++)
                            {
                                float dist=sqrt( (occ_crit_filt[c].x-opt_x)*(occ_crit_filt[c].x-opt_x)+(occ_crit_filt[c].y-opt_y)*(occ_crit_filt[c].y-opt_y) );
                                float angle=atan2(occ_crit_filt[c].y-opt_y, occ_crit_filt[c].x-opt_x);

                                float cos_ang=cos(angle);
                                float sin_ang=sin(angle);

                                float sign_x=0; if(cos_ang!=0) sign_x=cos_ang/abs(cos_ang);
                                float sign_y=0; if(sin_ang!=0) sign_y=sin_ang/abs(sin_ang);


                                int p_x=occ_crit_filt[c].x;
                                int p_y=occ_crit_filt[c].y;

                                float temp, tempx=occ_crit_filt[c].x, tempy=occ_crit_filt[c].y, temp_tx, temp_ty;

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

                                if(sign_x==0)
                                    p_x=p_x;
                                else
                                    p_x=sign_x*((int)round(sign_x*(tempx+temp*cos_ang)+p_x)-p_x);

                                if(sign_y==0)
                                    p_y=p_y;
                                else
                                    p_y=sign_y*((int)round(sign_y*(tempy+temp*sin_ang)+p_y)-p_y);

                                while( (dist+temp)<=defl && p_x>=0 && p_x<regions.rows && p_y>=0 && p_y<regions.cols )
                                {
                                    dist=dist+temp;

                                    vis_map_temp.at<uchar>(p_x,p_y)=0;

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

                                    if(sign_x==0)
                                        p_x=p_x;
                                    else
                                        p_x=sign_x*((int)round(sign_x*(tempx+temp*cos_ang)+p_x)-p_x);

                                    if(sign_y==0)
                                        p_y=p_y;
                                    else
                                        p_y=sign_y*((int)round(sign_y*(tempy+temp*sin_ang)+p_y)-p_y);
                                }
                            }

                            for(int j=0;j<frontier[ff].size();j++){
                                if(vis_map_temp.at<uchar>( frontier[ff][j].x,frontier[ff][j].y)==255)
                                {
                                    std::vector<cv::Point> points_vis=label_seed(vis_map_temp.clone()/255,4,cv::Point(frontier[ff][j].x,frontier[ff][j].y));
                                    for(int pv=0;pv<points_vis.size();pv++)
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

            map_label=l_map;
            map_act=act_map;
            map_vis=vis_map;

            map_debug=unreach_map;

            vector<cv::Mat> channels(3);

            channels[0]=map_erosionOp.clone();
            channels[1]=map_erosionOp.clone();
            channels[2]=map_erosionOp.clone();

            channels[1].at<uchar>(pos_x-1,pos_y-1)=255;
            channels[1].at<uchar>(pos_x-1,pos_y)=255;
            channels[1].at<uchar>(pos_x-1,pos_y+1)=255;
            channels[1].at<uchar>(pos_x,pos_y-1)=255;
            channels[1].at<uchar>(pos_x,pos_y)=255;
            channels[1].at<uchar>(pos_x,pos_y+1)=255;
            channels[1].at<uchar>(pos_x+1,pos_y-1)=255;
            channels[1].at<uchar>(pos_x+1,pos_y)=255;
            channels[1].at<uchar>(pos_x+1,pos_y+1)=255;

            channels[2].at<uchar>(pos_x-1,pos_y-1)=0;
            channels[2].at<uchar>(pos_x-1,pos_y)=0;
            channels[2].at<uchar>(pos_x-1,pos_y+1)=0;
            channels[2].at<uchar>(pos_x,pos_y-1)=0;
            channels[2].at<uchar>(pos_x,pos_y)=0;
            channels[2].at<uchar>(pos_x,pos_y+1)=0;
            channels[2].at<uchar>(pos_x+1,pos_y-1)=0;
            channels[2].at<uchar>(pos_x+1,pos_y)=0;
            channels[2].at<uchar>(pos_x+1,pos_y+1)=0;

            channels[0].at<uchar>(pos_x-1,pos_y-1)=0;
            channels[0].at<uchar>(pos_x-1,pos_y)=0;
            channels[0].at<uchar>(pos_x-1,pos_y+1)=0;
            channels[0].at<uchar>(pos_x,pos_y-1)=0;
            channels[0].at<uchar>(pos_x,pos_y)=0;
            channels[0].at<uchar>(pos_x,pos_y+1)=0;
            channels[0].at<uchar>(pos_x+1,pos_y-1)=0;
            channels[0].at<uchar>(pos_x+1,pos_y)=0;
            channels[0].at<uchar>(pos_x+1,pos_y+1)=0;

            cv::merge(channels, map_erosionOpPrintColor);



            ros::Duration diff = ros::Time::now() - t01;

            ROS_INFO("%s - Time for label: %f", tf_pref.c_str(), diff.toSec());

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

        prev_x=pos_x;
        prev_y=pos_y;
    }
    else
        return;


}

void Vis_transf::update(void)
{
    bool proc=false;

    mtx.lock();

    map_transform::ParametersConfig config=_config;

    if(changed_p)
    {
        changed_p=false;

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


std::vector<std::vector<cv::Point> >  Vis_transf::label(const cv::Mat binary, int conn)
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

std::vector<cv::Point> Vis_transf::label_seed(const cv::Mat binary, int conn, cv::Point seed)
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

    Vis_transf visNC(nh);

    ros::Rate loop_rate(10);


    while (ros::ok())
    {
        visNC.update();

        ros::spinOnce();

        visNC.transf();

        visNC.transf_pos();

        visNC.show();

        visNC.publish();

        loop_rate.sleep();
    }


    return 0;
}
