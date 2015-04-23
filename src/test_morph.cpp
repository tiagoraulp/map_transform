#include "ros/ros.h"
#include "string"
#include "nav_msgs/OccupancyGrid.h"
#include <unistd.h>

//#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/flann/miniflann.hpp"
//#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/ml/ml.hpp"
//#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#include "boost/thread/thread.hpp"
#include "boost/thread/mutex.hpp"
#include "boost/bind.hpp"

 #include <tf/transform_listener.h>


#include <sstream>

const double PI = 3.141592653589793;

bool debug;

static const std::string M_WINDOW = "Map";
static const std::string E_WINDOW = "Erosion";
static const std::string C_WINDOW = "Close";
static const std::string L_WINDOW = "Labelled";
static const std::string A_WINDOW = "Actuation";
static const std::string V_WINDOW = "Visibility";
static const std::string D_WINDOW = "Debug";
static const std::string G_WINDOW = "Ground_Truth";
static const std::string S_WINDOW = "Structuring Element";

using namespace std;

//boost::mutex mux;

class Reach_transf{
private:
    ros::NodeHandle nh_;

    ros::Publisher pub;
    ros::Publisher pub2;
    ros::Publisher pub5;
    ros::Publisher pub6;
    ros::Publisher pub7;
    ros::Publisher pub8;

    ros::Subscriber sub;

    tf::TransformListener pos_listener;

    std::vector<std::vector<cv::Point> >  label(const cv::Mat binary, int conn);
    std::vector<cv::Point> label_seed(const cv::Mat binary, int conn, cv::Point seed);
    nav_msgs::OccupancyGrid Mat2RosMsg(cv::Mat map ,const nav_msgs::OccupancyGrid& msg);
    //std::vector<std::vector<signed char> > close (std::vector<std::vector<signed char> > map,float radius);

    void rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    int count;

    bool pos_rcv;

    bool treated;

    int infl;

    int defl;

    int kernel;

    int prev_x, prev_y;

    string tf_pref;

    int height, width;

    float res, or_x, or_y;

    nav_msgs::OccupancyGrid msg_rcv,msg_rcv_pub;

    cv::Mat cv_map, map_or, map_erosionOp, map_closeOp, map_eroded_skel, map_reach, map_label , map_act, map_vis, map_debug, map_truth,struct_elem;

public:

    Reach_transf(ros::NodeHandle nh): nh_(nh)
    {

        pub = nh_.advertise<nav_msgs::OccupancyGrid>("e_map", 1,true);
        pub2 = nh_.advertise<nav_msgs::OccupancyGrid>("c_map", 1,true);
        pub5 = nh_.advertise<nav_msgs::OccupancyGrid>("vis_map", 1,true);
        pub6 = nh_.advertise<nav_msgs::OccupancyGrid>("a_map", 1,true);
        pub7 = nh_.advertise<nav_msgs::OccupancyGrid>("v_map", 1,true);
        pub8 = nh_.advertise<nav_msgs::OccupancyGrid>("g_map", 1,true);

        sub = nh_.subscribe("map", 1, &Reach_transf::rcv_map, this);

        nh_.param("infl", infl, 5);
        nh_.param("defl", defl, infl);

        nh_.param("kernel", kernel, 2);

        nh_.param("tf_prefix", tf_pref, std::string(""));

        count=0;

        treated=false;

        pos_rcv=false;



            cv::namedWindow(M_WINDOW);
            cv::namedWindow(E_WINDOW);
            cv::namedWindow(C_WINDOW);
            cv::namedWindow(S_WINDOW);
            cv::namedWindow(D_WINDOW);

         if(debug){
            cv::namedWindow(L_WINDOW);
            cv::namedWindow(A_WINDOW);
            cv::namedWindow(V_WINDOW);
            cv::namedWindow(G_WINDOW);

        }
    }

    ~Reach_transf()
    {

           cv::destroyWindow(M_WINDOW);
           cv::destroyWindow(E_WINDOW);
           cv::destroyWindow(C_WINDOW);
           cv::destroyWindow(S_WINDOW);
           cv::destroyWindow(D_WINDOW);

        if(debug){
           cv::destroyWindow(L_WINDOW);
           cv::destroyWindow(A_WINDOW);
           cv::destroyWindow(V_WINDOW);
           cv::destroyWindow(G_WINDOW);
        }
    }

    void show(void);
    void publish(void);
    void transf(void);
    void spin(void);


    void transf_pos(void);


    bool getTreated(void){return treated;}


};



void Reach_transf::show(void)
{

    //boost::mutex::scoped_lock lock(mux);
    if(count>0){


        cv::imshow(M_WINDOW,map_or);
        cv::imshow(E_WINDOW,map_erosionOp);
        cv::imshow(C_WINDOW,map_closeOp);

        cv::imshow(S_WINDOW,struct_elem);

        cv::imshow(D_WINDOW,map_debug);


        if(pos_rcv)
        {
            cv::imshow(L_WINDOW,map_label);
            cv::imshow(A_WINDOW,map_act);
            cv::imshow(V_WINDOW,map_vis);
            cv::imshow(G_WINDOW,map_truth);
        }
        cv::waitKey(3);
    }
}


nav_msgs::OccupancyGrid Reach_transf::Mat2RosMsg(cv::Mat map ,const nav_msgs::OccupancyGrid& msg)
{
    //boost::mutex::scoped_lock lock(mux);
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

void Reach_transf::rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    //boost::mutex::scoped_lock lock(mux);
    ROS_INFO("I heard map: [%d]", msg->header.seq);

    /////////////////////////////////
    //std::vector<std::vector<signed char> > GridMap(msg->info.height, std::vector<signed char>(msg->info.width, -1));

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
                //signed char val;
                unsigned char val_cv;
                if(*mapDataIterC >  map_occ_thres)
                {  /*val = 1;*/val_cv=0;}
                else if(*mapDataIterC == 0)
                {  /*val = 0;*/val_cv=255;}
                else
                {  /*val = -1;*/val_cv=255;}
                //GridMap[i][j]=val;
                cv_map.at<uchar>(j,i) = val_cv;

                if(count>0 && treated)
                    if(val_cv!=prev_map.at<uchar>(j,i))
                        treated=false;

                mapDataIterC++;
            }
    }

    ++count;

    msg_rcv=*msg;

    //treated=false;



    transf();


}


void Reach_transf::spin(void)
{
    while(ros::ok())
    {
        transf();
    }
}


cv::Mat color_print(cv::Mat img1, cv::Mat img2, unsigned char* c_b, unsigned char* c_n, unsigned char* c_1, unsigned char* c_2)
{
    cv::Mat result;// = cv::Mat(img1.rows, img1.cols, CV_8UC3), img1, img2;

    //map_debug.at<uchar>();

    //input1.convertTo(img1, CV_32FC1,1.0/255);
    //input2.convertTo(img2, CV_32FC1),1.0/255;

    vector<cv::Mat> channels(3);

    channels[0]=(img1&img2/255*c_b[2])+((~img1)&(~img2)/255*c_n[2])+((~img1)&img2/255*c_2[2])+(img1&(~img2)/255*c_1[2]);
    channels[1]=(img1&img2/255*c_b[1])+((~img1)&(~img2)/255*c_n[1])+((~img1)&img2/255*c_2[1])+(img1&(~img2)/255*c_1[1]);
    channels[2]=(img1&img2/255*c_b[0])+((~img1)&(~img2)/255*c_n[0])+((~img1)&img2/255*c_2[0])+(img1&(~img2)/255*c_1[0]);

    //channels[0]=img1&img2/255*125;
    //channels[1]=(~img1)&(~img2)/255*125;
    //channels[2]=(img1&img2/255*125)+((~img1)&(~img2)/255*125);

    cout<<"Test 123: \n"<<+channels[0].at<uchar>(100,100)<<"; "<<(int)channels[1].at<uchar>(100,100)<<"; "<<(int)channels[2].at<uchar>(100,100)<<endl;

    cv::merge(channels, result);

    //result.convertTo(result, CV_8UC3,1);

    return result;
}

cv::Mat color_print3(cv::Mat img1, cv::Mat img2, cv::Mat img3, unsigned char* c_123, unsigned char* c_12, unsigned char* c_13, unsigned char* c_23, unsigned char* c_1, unsigned char* c_2, unsigned char* c_3, unsigned char* c_0)
{
    cv::Mat result;// = cv::Mat(img1.rows, img1.cols, CV_8UC3), img1, img2;

    //map_debug.at<uchar>();

    //input1.convertTo(img1, CV_32FC1,1.0/255);
    //input2.convertTo(img2, CV_32FC1),1.0/255;

    vector<cv::Mat> channels(3);

    channels[0]=(img1&img2&img3/255*c_123[2])+(img1&img2&(~img3)/255*c_12[2])+(img1&(~img2)&img3/255*c_13[2])+((~img1)&img2&img3/255*c_23[2])+(img1&(~img2)&(~img3)/255*c_1[2])+((~img1)&img2&(~img3)/255*c_2[2])+((~img1)&(~img2)&img3/255*c_3[2])+((~img1)&(~img2)&(~img3)/255*c_0[2]);
    channels[1]=(img1&img2&img3/255*c_123[1])+(img1&img2&(~img3)/255*c_12[1])+(img1&(~img2)&img3/255*c_13[1])+((~img1)&img2&img3/255*c_23[1])+(img1&(~img2)&(~img3)/255*c_1[1])+((~img1)&img2&(~img3)/255*c_2[1])+((~img1)&(~img2)&img3/255*c_3[1])+((~img1)&(~img2)&(~img3)/255*c_0[1]);
    channels[2]=(img1&img2&img3/255*c_123[0])+(img1&img2&(~img3)/255*c_12[0])+(img1&(~img2)&img3/255*c_13[0])+((~img1)&img2&img3/255*c_23[0])+(img1&(~img2)&(~img3)/255*c_1[0])+((~img1)&img2&(~img3)/255*c_2[0])+((~img1)&(~img2)&img3/255*c_3[0])+((~img1)&(~img2)&(~img3)/255*c_0[0]);


    //channels[0]=img1&img2/255*125;
    //channels[1]=(~img1)&(~img2)/255*125;
    //channels[2]=(img1&img2/255*125)+((~img1)&(~img2)/255*125);

    //cout<<"Test 123: \n"<<+channels[0].at<uchar>(100,100)<<"; "<<(int)channels[1].at<uchar>(100,100)<<"; "<<(int)channels[2].at<uchar>(100,100)<<endl;

    cv::merge(channels, result);

    //result.convertTo(result, CV_8UC3,1);

    return result;
}


void Reach_transf::transf(void)
{
    //boost::mutex::scoped_lock lock(mux);



    //if (count>0 )//&& !treated)
    //{

        ros::Time t01=ros::Time::now();

        //treated=true;
        cv::Mat element1 = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                               cv::Size( 2*infl + 1, 2*infl+1 ),
                                               cv::Point( infl, infl ) );


        cv::Mat element2 = cv::getStructuringElement( cv::MORPH_ERODE,
                                               cv::Size( 2*infl + 1, 2*infl+1 ),
                                               cv::Point( infl, infl ) );

        cv::Mat element3 = cv::getStructuringElement( cv::MORPH_DILATE,
                                               cv::Size( 2*infl + 1, 2*infl+1 ),
                                               cv::Point( infl, infl ) );

        //cv::Mat element4 = cv::getStructuringElement( cv::MORPH_CLOSE,
        //                                       cv::Size( 2*infl + 1, 2*infl+1 ),
        //                                         cv::Point( infl, infl ) );


        cv::Mat element5 = cv::getStructuringElement( cv::MORPH_CROSS,
                                               cv::Size( 2*infl + 1, 2*infl+1 ),
                                               cv::Point( infl, infl ) );

        cv::Mat element6 = cv::getStructuringElement( cv::MORPH_RECT,
                                               cv::Size( 2*infl + 1, 2*infl+1 ),
                                               cv::Point( infl, infl ) );

        cv::Mat element11 = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                               cv::Size( 2*defl + 1, 2*defl+1 ),
                                               cv::Point( defl, defl ) );


        cv::Mat element12 = cv::getStructuringElement( cv::MORPH_ERODE,
                                               cv::Size( 2*defl + 1, 2*defl+1 ),
                                               cv::Point( defl, defl ) );

        cv::Mat element13 = cv::getStructuringElement( cv::MORPH_DILATE,
                                               cv::Size( 2*defl + 1, 2*defl+1 ),
                                               cv::Point( defl, defl ) );

        //cv::Mat element4 = cv::getStructuringElement( cv::MORPH_CLOSE,
        //                                       cv::Size( 2*infl + 1, 2*infl+1 ),
        //                                         cv::Point( infl, infl ) );


        cv::Mat element15 = cv::getStructuringElement( cv::MORPH_CROSS,
                                               cv::Size( 2*defl + 1, 2*defl+1 ),
                                               cv::Point( defl, defl ) );

        cv::Mat element16 = cv::getStructuringElement( cv::MORPH_RECT,
                                               cv::Size( 2*defl + 1, 2*defl+1 ),
                                               cv::Point( defl, defl ) );


        cv::Mat or_map, er_map, cl_map, es_map, r_map, temp;

        or_map=cv_map.clone();
        msg_rcv_pub=msg_rcv;


        /// Apply the erosion operation
        ///
        ///
        ///
        ///
        /// c



        unsigned char data[100]={0,0,0,0,0,0,1,0,0,0,
                                 0,0,0,0,0,1,1,1,0,0,
                                 0,0,0,0,1,1,1,1,1,0,
                                 0,0,0,1,1,1,1,1,1,1,
                                 0,0,1,1,1,1,1,1,1,0,
                                 0,1,1,1,1,1,1,1,0,0,
                                 1,1,1,1,1,1,1,0,0,0,
                                 0,1,1,1,1,1,0,0,0,0,
                                 0,0,1,1,1,0,0,0,0,0,
                                 0,0,0,1,0,0,0,0,0,0};
        cv::Mat elem = cv::Mat(10, 10, CV_8UC1, data);
//cv::Mat elem = cv::Mat::ones(10,10,CV_8U);
        switch (kernel) {
        case 1:
            //erode( or_map, er_map, element1);
            //dilate( er_map, cl_map, element1 );
            break;
        case 3:
            //erode( or_map, er_map, element5);
            //dilate( er_map, cl_map, element5 );
            break;
        default:
            //erode( or_map, er_map, element6);
            //dilate( er_map, cl_map, element6 );
            break;
        }

        //erode( or_map, er_map, element1);
        //erode( cv_map, map_erosionOp, element );
        //dilate( er_map, cl_map, element3 );
        //dilate( er_map, cl_map, element1 );

        //cv::morphologyEx(or_map,cl_map,cv::MORPH_CLOSE, element1);


        cv::erode( or_map, er_map, elem, cv::Point(6,3) );

        //cv::dilate( ~or_map, er_map, elem, cv::Point(3,6) );

        cv::Mat elem2;

        flip(elem, elem2, -1);

        cv::dilate( er_map, cl_map, elem2, cv::Point(9-6,9-3) );


        //map_debug = cv::Mat(or_map.rows, or_map.cols, CV_8UC3);

        //map_debug.at<uchar>();

        //vector<cv::Mat> channels(3);

        //channels[0]=er_map;
        //channels[1]=er_map;
        //channels[2]=cl_map;

        //merge(channels, map_debug);


        //erode( or_map, er_map, element1);

        unsigned char color_b[3]={255,255,0};
        unsigned char color_n[3]={0,0,255};
        unsigned char color_1[3]={255,0,0};
        unsigned char color_2[3]={0,255,0};

        unsigned char c123[3]={0,0,255};
        unsigned char c12[3]={0,0,0};
        unsigned char c13[3]={0,0,0};
        unsigned char c23[3]={0,255,255};
        unsigned char c1[3]={0,0,0};
        unsigned char c2[3]={0,0,0};
        unsigned char c3[3]={255,0,0};
        unsigned char c0[3]={0,0,0};

        //map_debug=color_print(er_map, cl_map, color_b , color_n , color_1 , color_2 );

        map_debug=color_print3(er_map, cl_map, or_map, c123, c12, c13, c23, c1, c2, c3, c0 );

        map_or=or_map;
        map_erosionOp=er_map;
        map_closeOp=cl_map;

        struct_elem=elem*255;

        vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(9);

        cv::imshow("Test!!!!",map_debug);
        cv::waitKey(30);

        try {
            cv::imwrite("/home/tiago/map_debug.png", map_debug, compression_params);

        }
        catch (exception& e)
        {
          cout  <<"Exception: "<< e.what() << '\n';
        }



        ros::Duration diff = ros::Time::now() - t01;

        cout<<tf_pref<<" - Time for reach: "<<diff<<endl;

    //}


}


bool raytracing(cv::Mat map, int opt_x, int opt_y, int dest_x, int dest_y)
{
    bool debug=false;

    if(opt_x==0 && opt_y==145)
        debug=false;

    float dist_t=sqrt( (dest_x-opt_x)*(dest_x-opt_x)+(dest_y-opt_y)*(dest_y-opt_y) );
    float angle=atan2(dest_y-opt_y, dest_x-opt_x);

    float cos_ang=cos(angle);
    float sin_ang=sin(angle);

    float sign_x=0; if(cos_ang!=0) sign_x=cos_ang/abs(cos_ang);
    float sign_y=0; if(sin_ang!=0) sign_y=sin_ang/abs(sin_ang);


    int p_x=opt_x;
    int p_y=opt_y;

    if(debug)
        cout<<"Test!!!!!!!!!!!!!!!!!"<<endl<<p_x<<" "<<p_y<<endl;
    if(debug)
        cout<<angle<<" "<<cos_ang<<" "<<sin_ang<<endl;
    if(debug)
        cout<<dist_t<<endl;

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

    //                                bool count_free=false;

    //cout<<p_x<<" "<<p_y<<" "<<sign_x*(tempx+temp*cos_ang)<<" "<<sign_y*(tempy+temp*sin_ang)<<" "<<tempx<<" "<<tempy<<" "<<dist+temp<<" "<<dist<<endl;

    int n=1;

    while( (dist+temp)<=dist_t && p_x>=0 && p_x<map.rows && p_y>=0 && p_y<map.cols )
    {
        n++;

        dist=dist+temp;

        if(debug)
            cout<<p_x<<" "<<p_y<<endl;

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


        //if(n<50)
            //cout<<p_x<<" "<<p_y<<" "<<sign_x*(tempx+temp*cos_ang)+p_x<<" "<<sign_y*(tempy+temp*sin_ang)+p_y<<" "<<tempx<<" "<<tempy<<" "<<dist+temp<<" "<<dist<<endl;

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

    if(debug)
        cout<<"Found!!!!!"<<endl;

    return true;
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


void Reach_transf::transf_pos(void)
{



    if(count>0)
    {
        tf::StampedTransform transform;
        try{
            pos_listener.lookupTransform("/map", tf_pref+"/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_INFO("%s",ex.what());



          return ;
        }



        ros::Time t01=ros::Time::now();

        int pos_x=(int) round((transform.getOrigin().x()-or_x)/res);

        int pos_y=(int) round((transform.getOrigin().y()-or_y)/res);

        if(map_erosionOp.at<uchar>(pos_x,pos_y)==0)
        {
            pos_rcv=false;

            prev_x=pos_x;
            prev_y=pos_y;

            treated=false;
            return;

        }

        //cout<<pos_x<<" ; "<<pos_y<<endl;

        cv::Mat temp_labelling, temp;
        //bitwise_not( map_erosionOp.clone() , temp_labelling);

        std::vector<std::vector<cv::Point> > labels=label(map_erosionOp.clone()/255,8);


        int label_pos=-1, prev_label=-1;


        for (int i=0;i<labels.size();i++){
            for(int j=0;j<labels[i].size();j++){
                //ROS_INFO("%d-%d ; %d-%d",i+1, labels.size(),j+1,labels[i].size());

                //ROS_INFO("%d %d",labels[i][j].x, labels[i][j].y);
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

        if( (prev_label!=label_pos) || ! treated)
        {


            cv::Mat l_map=map_erosionOp.clone(), unreach_map, regions;


            for (int i=0;i<labels.size();i++){
                for(int j=0;j<labels[i].size();j++){
                    if( i!=label_pos ){

                        l_map.at<uchar>(labels[i][j].x,labels[i][j].y)=0;

                    }
                }
             }

            //cout<<"Test"<<endl;


            cv::Mat element1 = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                                   cv::Size( 2*infl + 1, 2*infl+1 ),
                                                   cv::Point( infl, infl ) );


            cv::Mat element2 = cv::getStructuringElement( cv::MORPH_ERODE,
                                                   cv::Size( 2*infl + 1, 2*infl+1 ),
                                                   cv::Point( infl, infl ) );

            cv::Mat element3 = cv::getStructuringElement( cv::MORPH_DILATE,
                                                   cv::Size( 2*infl + 1, 2*infl+1 ),
                                                   cv::Point( infl, infl ) );

            //cv::Mat element4 = cv::getStructuringElement( cv::MORPH_CLOSE,
            //                                       cv::Size( 2*infl + 1, 2*infl+1 ),
            //                                         cv::Point( infl, infl ) );


            cv::Mat element5 = cv::getStructuringElement( cv::MORPH_CROSS,
                                                   cv::Size( 2*infl + 1, 2*infl+1 ),
                                                   cv::Point( infl, infl ) );

            cv::Mat element6 = cv::getStructuringElement( cv::MORPH_RECT,
                                                   cv::Size( 2*infl + 1, 2*infl+1 ),
                                                   cv::Point( infl, infl ) );

            int rad=min(infl,defl);


            cv::Mat element11 = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                                   cv::Size( 2*rad + 1, 2*rad+1 ),
                                                   cv::Point( rad, rad ) );


            cv::Mat element12 = cv::getStructuringElement( cv::MORPH_ERODE,
                                                   cv::Size( 2*rad + 1, 2*rad+1 ),
                                                   cv::Point( rad, rad ) );

            cv::Mat element13 = cv::getStructuringElement( cv::MORPH_DILATE,
                                                   cv::Size( 2*rad + 1, 2*rad+1 ),
                                                   cv::Point( rad, rad ) );

            //cv::Mat element4 = cv::getStructuringElement( cv::MORPH_CLOSE,
            //                                       cv::Size( 2*infl + 1, 2*infl+1 ),
            //                                         cv::Point( infl, infl ) );


            cv::Mat element15 = cv::getStructuringElement( cv::MORPH_CROSS,
                                                   cv::Size( 2*rad + 1, 2*rad+1 ),
                                                   cv::Point( rad, rad ) );

            cv::Mat element16 = cv::getStructuringElement( cv::MORPH_RECT,
                                                   cv::Size( 2*rad + 1, 2*rad+1 ),
                                                   cv::Point( rad, rad ) );




            /// Apply the erosion operation
            ///
            ///
            ///
            ///
            ///
            ///

            cv::Mat act_map=l_map.clone();


            switch (kernel) {
            case 1:
                //erode( or_map, er_map, element1);
                dilate( act_map, act_map, element11 );
                break;
            case 3:
                //erode( or_map, er_map, element5);
                dilate( act_map, act_map,  element15 );
                break;
            default:
                //erode( or_map, er_map, element6);
                dilate( act_map, act_map,  element16 );
                break;
            }

            cv::Mat vis_map=act_map.clone(), vis_map_temp, contours;


            bitwise_not(map_or,temp);

            unreach_map=vis_map|temp;

            bitwise_not( unreach_map.clone() , temp_labelling);

            std::vector<std::vector<cv::Point> > labels_unreach=label(temp_labelling/255,4);


            //label_pos=2;


            regions=map_or.clone()/255;

            for (int i=0;i<labels_unreach.size();i++){
                for(int j=0;j<labels_unreach[i].size();j++){
                    //if( i!=label_pos ){

                        regions.at<uchar>(labels_unreach[i][j].x,labels_unreach[i][j].y)=i+2;

                    //}
                }
             }

            //int k=label_pos;


            //vis_map=vis_map_temp;

            //=unreach_map;

            map_label=l_map;
            map_act=act_map;
            map_vis=vis_map;

            //map_debug=unreach_map;
            //map_debug=contours;


            ros::Duration diff = ros::Time::now() - t01;

            cout<<tf_pref<<" - Time for label: "<<diff<<endl;


            ros::Time t3=ros::Time::now();

            map_truth=brute_force_opt(map_or, map_label, defl);

            diff = ros::Time::now() - t3;

            cout<<tf_pref<<" - Time for Optimized brute force: "<<diff<<endl;

        }



        pos_rcv=true;

        prev_x=pos_x;
        prev_y=pos_y;

        treated=true;

    }
    else
        return;


}


void Reach_transf::publish(void)
{
    //boost::mutex::scoped_lock lock(mux);

        //////////////////////////////////

        nav_msgs::OccupancyGrid n_msg;


        n_msg=Mat2RosMsg(map_erosionOp , msg_rcv_pub);
        pub.publish(n_msg);

        n_msg=Mat2RosMsg(map_closeOp , msg_rcv_pub);
        pub2.publish(n_msg);



        if(pos_rcv)
        {
            n_msg=Mat2RosMsg( map_label , msg_rcv_pub);
            pub5.publish(n_msg);
            n_msg=Mat2RosMsg( map_act , msg_rcv_pub);
            pub6.publish(n_msg);
            n_msg=Mat2RosMsg( map_vis , msg_rcv_pub);
            pub7.publish(n_msg);
            n_msg=Mat2RosMsg( map_truth , msg_rcv_pub);
            pub8.publish(n_msg);
        }


        std::stringstream ss;
        ss << tf_pref+"-> Message sent: " << count;
        //msg.data = ss.str();
        //ROS_INFO("%s", ss.str().c_str());


}


std::vector<std::vector<cv::Point> >  Reach_transf::label(const cv::Mat binary, int conn)
{
    std::vector<std::vector<cv::Point> > blobs;
    blobs.clear();

    // Using labels from 2+ for each blob
    cv::Mat label_image;
    binary.convertTo(label_image, CV_32FC1);

    //ROS_INFO("%f %f %f %f", label_image.at<float>(0,0), label_image.at<float>(100,100), label_image.at<float>(100,150), label_image.at<float>(150,50) );


    //cv::imshow("test",label_image);

    int label_count = 2; // starts at 2 because 0,1 are used already

    //if(pos_rcv)
    //    ROS_INFO("%f %f %f %f", label_image.at<float>(0,0), label_image.at<float>(50,50), label_image.at<float>(100,150), label_image.at<float>(150,50) );
        //ROS_INFO("label %d", label_count);




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

            //ROS_INFO("size %d", blob.size());

            //if(pos_rcv)
            //    ROS_INFO("%f %f %f %f", label_image.at<float>(0,0), label_image.at<float>(50,50), label_image.at<float>(100,150), label_image.at<float>(150,50) );
                //ROS_INFO("label %d", label_count);




            label_count++;
        }
    }

    return blobs;
}

std::vector<cv::Point> Reach_transf::label_seed(const cv::Mat binary, int conn, cv::Point seed)
{
    std::vector<cv::Point> blob;
    blob.clear();

    // Using labels from 2+ for each blob
    cv::Mat label_image;
    binary.convertTo(label_image, CV_32FC1);

    //ROS_INFO("%f %f %f %f", label_image.at<float>(0,0), label_image.at<float>(100,100), label_image.at<float>(100,150), label_image.at<float>(150,50) );


    //cv::imshow("test",label_image);

    int label_count = 2; // starts at 2 because 0,1 are used already

    //if(pos_rcv)
    //    ROS_INFO("%f %f %f %f", label_image.at<float>(0,0), label_image.at<float>(50,50), label_image.at<float>(100,150), label_image.at<float>(150,50) );
        //ROS_INFO("label %d", label_count);





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


            //ROS_INFO("size %d", blob.size());

            //if(pos_rcv)
            //    ROS_INFO("%f %f %f %f", label_image.at<float>(0,0), label_image.at<float>(50,50), label_image.at<float>(100,150), label_image.at<float>(150,50) );
                //ROS_INFO("label %d", label_count);




    return blob;
}



int main(int argc, char **argv)
{



    ros::init(argc, argv, "reach");
    debug=true;
    if(argc==2)
        if(string(argv[1])==string("0"))
            debug=false;


  ros::NodeHandle nh("~");


//  vector<cv::Point> frontier[ff]s;frontier[ff]s.clear();
//  cv::Point p;
//  p.x=1;p.y=1;
//  frontier[ff]s.push_back(p);
//  p.x=16;p.y=16;
//  frontier[ff]s.push_back(p);
//  p.x=1;p.y=2;
//  frontier[ff]s.push_back(p);
//  p.x=16;p.y=15;
//  frontier[ff]s.push_back(p);
//  p.x=1;p.y=3;
//  frontier[ff]s.push_back(p);
//  p.x=15;p.y=14;
//  frontier[ff]s.push_back(p);
//  p.x=2;p.y=4;
//  frontier[ff]s.push_back(p);
//  p.x=16;p.y=13;
//  frontier[ff]s.push_back(p);


//  vector<vector<cv::Point> > res=cluster_points(frontier[ff]s);

//  for(int i=0;i<res.size();i++)
//  {
//      cout<<"Cluster "<<i+1<<":"<<endl;
//      for(int j=0;j<res[i].size();j++)
//      {
//          cout<<res[i][j].x<<" "<<res[i][j].y<<"; ";
//      }
//      cout<<endl;
//  }



  Reach_transf reach(nh);

  //ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("n_map", 1);

  //ros::Subscriber sub = nh.subscribe("map", 1, &Reach_transf::transf, &reach);

  ros::Rate loop_rate(10);

  //boost::thread t(&Reach_transf::spin, &reach);

  while (ros::ok())
  {


    ros::spinOnce();

    //if (t.timed_join(boost::posix_time::seconds(0)) && !reach.getTreated() )
        //t.start_thread();
        //t=boost::thread::Thread(&Reach_transf::transf, &reach);

    //reach.transf_pos();

    //if(debug)
        reach.show();

    reach.publish();

    //reach.publish();

    loop_rate.sleep();
  }


  return 0;
}
