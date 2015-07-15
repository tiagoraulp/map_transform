#include "visNC_transf.hpp"

#include <map_transform/ParametersncConfig.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "labelling.hpp"
#include "ray.hpp"
#include "color.hpp"

using namespace std;

static const double PI = 3.141592653589793;

static const std::string M_WINDOW = "Map";
static const std::string E_WINDOW = "Erosion";
static const std::string C_WINDOW = "Close";
static const std::string L_WINDOW = "Labelled";
static const std::string A_WINDOW = "Actuation";
static const std::string V_WINDOW = "Visibility";
static const std::string D_WINDOW = "Debug";
static const std::string G_WINDOW = "Ground_Truth";
static const std::string S_WINDOW = "Structuring Element";



template<typename T>
VisNC_transf<T>::VisNC_transf(ros::NodeHandle nh, cv::Mat rob): Vis_transf<T>(nh), robot(rob)
{

    this->nh_.param("rinfl", rinfl, 100.0);
    rinfl/=100;
    this->nh_.param("sdefl", sdefl, 100.0);
    sdefl/=100;
    this->nh_.param("rx", rcx, 50.0);
    rcx/=100;
    this->nh_.param("ry", rcy, 50.0);
    rcy/=100;
    this->nh_.param("rt", rct, 0.0);

    this->nh_.param("x", this->rxr, 10.0);
    this->nh_.param("y", this->ryr, 10.0);
    this->nh_.param("theta", this->rtr, 0.0);

    this->nh_.param("ground_truth", this->gt, false);

    this->nh_.param("angle_res", this->angle_res, 32);

    this->nh_.param("debug_angle", this->angle_debug, 0.0);

    this->nh_.param("debug", this->_debug, true);

    this->nh_.param("tf_prefix", this->tf_pref, std::string(""));

    this->func = boost::bind(&VisNC_transf::callbackParameters, this,_1, _2);
    this->server.setCallback(this->func);


    this->treated=true;
    this->treated2=true;

    this->pos_rcv=false;

    this->gt_c=false;

    this->changed=false;
    this->changed2=false;

    this->changed_p=false;


    if(this->_debug){
        cv::namedWindow(M_WINDOW);
        cv::namedWindow(E_WINDOW);
        cv::namedWindow(C_WINDOW);
        cv::namedWindow(S_WINDOW);
        cv::namedWindow(D_WINDOW);
        if(this->pos_rcv)
        {
            cv::namedWindow(L_WINDOW);
            cv::namedWindow(A_WINDOW);
            cv::namedWindow(V_WINDOW);
            if(this->gt && this->gt_c)
                cv::namedWindow(G_WINDOW);
        }
    }


}

template<typename T>
VisNC_transf<T>::~VisNC_transf()
{
    if(this->_debug){
       cv::destroyWindow(M_WINDOW);
       cv::destroyWindow(E_WINDOW);
       cv::destroyWindow(C_WINDOW);
       cv::destroyWindow(S_WINDOW);
       cv::destroyWindow(D_WINDOW);
       cv::destroyWindow(L_WINDOW);
       cv::destroyWindow(A_WINDOW);
       cv::destroyWindow(V_WINDOW);
       cv::destroyWindow(G_WINDOW);

    }
}


template <typename T>
void VisNC_transf<T>::show(void)
{

    if(this->count>0 && this->_debug){


        cv::imshow(M_WINDOW,this->map_or);
        cv::imshow(E_WINDOW,this->map_erosionOpPrintColor);
        cv::imshow(C_WINDOW,this->map_closeOp);

        cv::imshow(S_WINDOW,this->struct_elem);

        cv::imshow(D_WINDOW,this->map_debug);


        if(this->pos_rcv)
        {
            cv::imshow(L_WINDOW,this->map_label);
            cv::imshow(A_WINDOW,this->map_act);
            cv::imshow(V_WINDOW,this->map_vis);
            if(this->gt && this->gt_c)
            {
                cv::imshow(G_WINDOW,this->map_truth);
            }
            else
            {
                cv::destroyWindow(G_WINDOW);
                cv::waitKey(2);
            }
        }
        cv::waitKey(3);
    }

    if(!this->_debug)
    {
       cv::waitKey(2);
       cv::destroyWindow(M_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(E_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(C_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(S_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(D_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(L_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(A_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(V_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(G_WINDOW);
       cv::waitKey(2);
    }
}

template <typename T>
void VisNC_transf<T>::transf(void)
{

    bool proc=false;


    if((!this->treated2) || this->changed2)
    {
        proc=true;
        this->treated2=true;
        this->changed2=false;
    }


    if(this->count>0 && (proc) )
    {
        ros::Time t01=ros::Time::now();

        cv::Mat or_map, er_map, cl_map, or_mapN;

        or_map=this->cv_map.clone();
        this->msg_rcv_pub=this->msg_rcv;


        cv::Mat elem = this->robot.clone();

        cv::Point2f pt=cv::Point2f(elem.cols*rcx,elem.rows*rcy);

        robot_or=multiElem(elem, pt,this->rct, this->rinfl, this->angle_res);


        cv::copyMakeBorder(or_map,or_mapN,this->robot_or.pu,this->robot_or.pb,this->robot_or.pl,robot_or.pr,cv::BORDER_CONSTANT,cv::Scalar(0));



        vector<cv::Mat> mer_map=multiErosion(or_mapN, robot_or);



        vector<cv::Mat> mcl_map=multiDilation(mer_map, robot_or);


        unsigned char c123[3]={0,0,255};
        unsigned char c12[3]={255,255,0};
        unsigned char c13[3]={255,0,255};
        unsigned char c23[3]={0,255,255};
        unsigned char c1[3]={0,255,0};
        unsigned char c2[3]={255,255,255};
        unsigned char c3[3]={255,0,0};
        unsigned char c0[3]={0,0,0};


        double rtrd=angle_debug;

        if(rtrd<0)
            rtrd+=360;

        double min_err;
        int m_a=0;

        for(int i=0;i<angle_res;i++)
        {
            if(i==0)
            {
                m_a=i;
                min_err=abs(360.0/angle_res*i-rtrd);
            }
            else
            {
                if( abs(360.0/angle_res*i-rtrd)<min_err  )
                {
                    m_a=i;
                    min_err=abs(360.0/angle_res*i-rtrd);
                }
            }

        }

        er_map=mer_map[m_a].clone();
        cl_map=mcl_map[m_a].clone();


        this->map_debug=color_print3(er_map, cl_map, or_mapN, c123, c12, c13, c23, c1, c2, c3, c0 );

        this->map_or=or_map;
        this->map_erosionOp=er_map;

        cv::Rect rec(robot_or.pl,robot_or.pu, or_map.cols, or_map.rows);
        this->map_closeOp=cl_map(rec);

        struct_elem=robot_or.elems[m_a]*255;

        ros::Duration diff = ros::Time::now() - t01;

        ROS_INFO("%s - Time for configuration space: %f", this->tf_pref.c_str(), diff.toSec());

    }

}


template <typename T>
void VisNC_transf<T>::transf_pos(void)
{

    if(this->count>0)
    {
        ros::Time t01=ros::Time::now();

        double xx,yy, tt;

        if(!this->_debug)
        {
            tf::StampedTransform transform;
            try{
                this->pos_listener.lookupTransform("/map", this->tf_pref+"/base_link", ros::Time(0), transform);
            }
            catch (tf::TransformException ex){
              ROS_INFO("%s",ex.what());
              return ;
            }
            xx=transform.getOrigin().x();
            yy=transform.getOrigin().y();
            tt=tf::getYaw(transform.getRotation())/2/PI*360;

        }
        else
        {
            xx=this->rxr;
            yy=this->ryr;
            tt=rtr;
        }

        if(tt<0)
            tt+=360;




        int pos_x=((int) round((xx-this->or_x)/this->res))+this->robot_or.pu;

        int pos_y=((int) round((yy-this->or_y)/this->res))+this->robot_or.pl;

        if(pos_x>=this->map_or.rows)
            pos_x=this->map_or.rows-1;
        if(pos_y>=this->map_or.cols)
            pos_y=this->map_or.cols-1;

        if(pos_x<0)
            pos_x=0;
        if(pos_y<0)
            pos_y=0;


        bool proc=false;

        if((!(this->treated)) || (this->changed))
        {
            proc=true;
            this->treated=true;
            this->changed=false;
        }

        int prev_x=this->prev.x;
        int prev_y=this->prev.y;

        cv::Mat mape=this->map_erosionOp.clone();

        if(mape.at<uchar>(pos_x,pos_y)==0 )
        {
            this->gt_c=false;

            this->pos_rcv=true;

            vector<cv::Mat> channels(3);

            channels[0]=this->map_erosionOp.clone();
            channels[1]=this->map_erosionOp.clone();
            channels[2]=this->map_erosionOp.clone();

            channels[2].at<uchar>(max(pos_x-1,0),max(pos_y-1,0))=255;
            channels[2].at<uchar>(max(pos_x-1,0),pos_y)=255;
            channels[2].at<uchar>(max(pos_x-1,0),min(pos_y+1,this->map_erosionOp.cols-1))=255;
            channels[2].at<uchar>(pos_x,max(pos_y-1,0))=255;
            channels[2].at<uchar>(pos_x,pos_y)=255;
            channels[2].at<uchar>(pos_x,min(pos_y+1,this->map_erosionOp.cols-1))=255;
            channels[2].at<uchar>(min(pos_x+1,this->map_erosionOp.rows-1),max(pos_y-1,0))=255;
            channels[2].at<uchar>(min(pos_x+1,this->map_erosionOp.rows-1),pos_y)=255;
            channels[2].at<uchar>(min(pos_x+1,this->map_erosionOp.rows-1),min(pos_y+1,this->map_erosionOp.cols-1))=255;

            channels[1].at<uchar>(max(pos_x-1,0),max(pos_y-1,0))=0;
            channels[1].at<uchar>(max(pos_x-1,0),pos_y)=0;
            channels[1].at<uchar>(max(pos_x-1,0),min(pos_y+1,this->map_erosionOp.cols-1))=0;
            channels[1].at<uchar>(pos_x,max(pos_y-1,0))=0;
            channels[1].at<uchar>(pos_x,pos_y)=0;
            channels[1].at<uchar>(pos_x,min(pos_y+1,this->map_erosionOp.cols-1))=0;
            channels[1].at<uchar>(min(pos_x+1,this->map_erosionOp.rows-1),max(pos_y-1,0))=0;
            channels[1].at<uchar>(min(pos_x+1,this->map_erosionOp.rows-1),pos_y)=0;
            channels[1].at<uchar>(min(pos_x+1,this->map_erosionOp.rows-1),min(pos_y+1,this->map_erosionOp.cols-1))=0;

            channels[0].at<uchar>(max(pos_x-1,0),max(pos_y-1,0))=0;
            channels[0].at<uchar>(max(pos_x-1,0),pos_y)=0;
            channels[0].at<uchar>(max(pos_x-1,0),min(pos_y+1,this->map_erosionOp.cols-1))=0;
            channels[0].at<uchar>(pos_x,max(pos_y-1,0))=0;
            channels[0].at<uchar>(pos_x,pos_y)=0;
            channels[0].at<uchar>(pos_x,min(pos_y+1,this->map_erosionOp.cols-1))=0;
            channels[0].at<uchar>(min(pos_x+1,this->map_erosionOp.rows-1),max(pos_y-1,0))=0;
            channels[0].at<uchar>(min(pos_x+1,this->map_erosionOp.rows-1),pos_y)=0;
            channels[0].at<uchar>(min(pos_x+1,this->map_erosionOp.rows-1),min(pos_y+1,this->map_erosionOp.cols-1))=0;


            cv::merge(channels, this->map_erosionOpPrintColor);


            this->map_label=cv::Mat::zeros(this->map_erosionOp.rows, this->map_erosionOp.cols, CV_8UC1);
            this->map_act=cv::Mat::zeros(this->map_or.rows, this->map_or.cols, CV_8UC1);
            this->map_vis=cv::Mat::zeros(this->map_or.rows, this->map_or.cols, CV_8UC1);



            if (prev_x>=0 && prev_y>=0 && prev_x<this->map_erosionOp.rows && prev_y<this->map_erosionOp.cols)
                if (mape.at<uchar>(prev_x,prev_y)==0 && !proc)
                {
                    prev_x=pos_x;
                    prev_y=pos_y;
                    this->prev.x=prev_x;
                    this->prev.y=prev_y;
                    return;
                }

            prev_x=pos_x;
            prev_y=pos_y;

            this->prev.x=prev_x;
            this->prev.y=prev_y;


            ros::Duration diff = ros::Time::now() - t01;

            ROS_INFO("%s - Time for visibility (invalid position): %f", this->tf_pref.c_str(), diff.toSec());

            return;

        }


        cv::Mat temp_labelling, temp;

        std::vector<std::vector<cv::Point> > labels=label(this->map_erosionOp.clone()/255,8);


        unsigned int label_pos=0, prev_label=0;
        bool found_pos=false, found_prev=false;




        for (unsigned int i=0;i<labels.size();i++){
            for(unsigned int j=0;j<labels[i].size();j++){
                if( pos_x==labels[i][j].x && pos_y==labels[i][j].y){
                    label_pos=i+1;
                    found_pos=true;
                }
                if(prev_x>=0 &&  prev_y>=0)
                {
                    if( prev_x==labels[i][j].x && prev_y==labels[i][j].y){
                        prev_label=i+1;
                        found_prev=true;
                    }
                }
                if(found_pos && ( (found_prev) || (prev_x<0) || (prev_y<0) ) )
                    break;
            }
            if(found_pos && ( (found_prev) || (prev_x<0) || (prev_y<0) ) )
                break;
        }

        if( ( (prev_label!=label_pos) && found_pos) || (prev_x<0) || (prev_y<0) || proc)
        {
            cv::Mat l_map=this->map_erosionOp.clone(), unreach_map, regions, act_map=this->map_or.clone(), vis_map=this->map_or.clone();

            for (unsigned int i=0;i<labels.size();i++){
                if( i!=(label_pos-1) ){
                    for(unsigned int j=0;j<labels[i].size();j++){
                        l_map.at<uchar>(labels[i][j].x,labels[i][j].y)=0;
                    }
                }
            }

            this->map_label=l_map;
            this->map_act=act_map;
            this->map_vis=vis_map;


            vector<cv::Mat> channels(3);

            channels[0]=this->map_erosionOp.clone();
            channels[1]=this->map_erosionOp.clone();
            channels[2]=this->map_erosionOp.clone();

            channels[1].at<uchar>(max(pos_x-1,0),max(pos_y-1,0))=255;
            channels[1].at<uchar>(max(pos_x-1,0),pos_y)=255;
            channels[1].at<uchar>(max(pos_x-1,0),min(pos_y+1,this->map_erosionOp.cols-1))=255;
            channels[1].at<uchar>(pos_x,max(pos_y-1,0))=255;
            channels[1].at<uchar>(pos_x,pos_y)=255;
            channels[1].at<uchar>(pos_x,min(pos_y+1,this->map_erosionOp.cols-1))=255;
            channels[1].at<uchar>(min(pos_x+1,this->map_erosionOp.rows-1),max(pos_y-1,0))=255;
            channels[1].at<uchar>(min(pos_x+1,this->map_erosionOp.rows-1),pos_y)=255;
            channels[1].at<uchar>(min(pos_x+1,this->map_erosionOp.rows-1),min(pos_y+1,this->map_erosionOp.cols-1))=255;

            channels[2].at<uchar>(max(pos_x-1,0),max(pos_y-1,0))=0;
            channels[2].at<uchar>(max(pos_x-1,0),pos_y)=0;
            channels[2].at<uchar>(max(pos_x-1,0),min(pos_y+1,this->map_erosionOp.cols-1))=0;
            channels[2].at<uchar>(pos_x,max(pos_y-1,0))=0;
            channels[2].at<uchar>(pos_x,pos_y)=0;
            channels[2].at<uchar>(pos_x,min(pos_y+1,this->map_erosionOp.cols-1))=0;
            channels[2].at<uchar>(min(pos_x+1,this->map_erosionOp.rows-1),max(pos_y-1,0))=0;
            channels[2].at<uchar>(min(pos_x+1,this->map_erosionOp.rows-1),pos_y)=0;
            channels[2].at<uchar>(min(pos_x+1,this->map_erosionOp.rows-1),min(pos_y+1,this->map_erosionOp.cols-1))=0;

            channels[0].at<uchar>(max(pos_x-1,0),max(pos_y-1,0))=0;
            channels[0].at<uchar>(max(pos_x-1,0),pos_y)=0;
            channels[0].at<uchar>(max(pos_x-1,0),min(pos_y+1,this->map_erosionOp.cols-1))=0;
            channels[0].at<uchar>(pos_x,max(pos_y-1,0))=0;
            channels[0].at<uchar>(pos_x,pos_y)=0;
            channels[0].at<uchar>(pos_x,min(pos_y+1,this->map_erosionOp.cols-1))=0;
            channels[0].at<uchar>(min(pos_x+1,this->map_erosionOp.rows-1),max(pos_y-1,0))=0;
            channels[0].at<uchar>(min(pos_x+1,this->map_erosionOp.rows-1),pos_y)=0;
            channels[0].at<uchar>(min(pos_x+1,this->map_erosionOp.rows-1),min(pos_y+1,this->map_erosionOp.cols-1))=0;


            cv::merge(channels, this->map_erosionOpPrintColor);

            ros::Duration diff = ros::Time::now() - t01;

            ROS_INFO("%s - Time for visibility: %f", this->tf_pref.c_str(), diff.toSec());

            if(this->gt)
            {
                ros::Time t3=ros::Time::now();

                this->map_truth=brute_force_opt(this->map_or, this->map_label, this->sdefl*30);

                diff = ros::Time::now() - t3;

                ROS_INFO("%s - Time for Optimized brute force: %f", this->tf_pref.c_str(), diff.toSec());

                this->gt_c=true;
            }

        }

        this->pos_rcv=true;

        prev_x=pos_x;
        prev_y=pos_y;

        this->prev.x=prev_x;
        this->prev.y=prev_y;
    }
    else
        return;
}

template <typename T>
void VisNC_transf<T>::update(void)
{
    bool proc=false;

    mtx.lock();

    T config=this->_config;

    if(this->changed_p)
    {
        this->changed_p=false;

        proc=true;
    }

    mtx.unlock();

    if(proc)
    {
        this->changed=true;
        this->changed2=true;
        rinfl=config.rinfl/100;
        sdefl=config.sdefl/100;
        rcx=config.rx/100;
        rcy=config.ry/100;
        rct=config.rt;
        angle_res=config.angle_res;
        this->_debug=config.debug;
        this->gt=config.ground_truth;
        angle_debug=config.debug_angle;
        this->rxr=config.x;
        this->ryr=config.y;
        rtr=config.theta;
    }

}

template class VisNC_transf<map_transform::ParametersncConfig>;
