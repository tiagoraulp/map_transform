#include "ros/ros.h"
#include "string"
#include "nav_msgs/OccupancyGrid.h"

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
static const std::string ES_WINDOW = "Erosion+Skeleton";
static const std::string R_WINDOW = "Reachable";
static const std::string L_WINDOW = "Labelled";
static const std::string A_WINDOW = "Actuation";
static const std::string V_WINDOW = "Visibility";
static const std::string D_WINDOW = "Debug";
static const std::string G_WINDOW = "Ground_Truth";

using namespace std;

//boost::mutex mux;

class Reach_transf{
private:
    ros::NodeHandle nh_;

    ros::Publisher pub;
    ros::Publisher pub2;
    ros::Publisher pub3;
    ros::Publisher pub4;
    ros::Publisher pub5;
    ros::Publisher pub6;
    ros::Publisher pub7;
    ros::Publisher pub8;
    
    ros::Subscriber sub;

    tf::TransformListener pos_listener;

    std::vector<std::vector<cv::Point> >  label(const cv::Mat binary, int conn);
    std::vector<cv::Point> label_seed(const cv::Mat binary, int conn, cv::Point seed);
    cv::Mat skel (cv::Mat img);
    cv::Mat skel2 (cv::Mat img);
    cv::Mat skel3 (cv::Mat img);
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

    cv::Mat cv_map, map_or, map_erosionOp, map_closeOp, map_eroded_skel, map_reach, map_label , map_act, map_vis, map_debug, map_truth;

public:

    Reach_transf(ros::NodeHandle nh): nh_(nh)
    {

        pub = nh_.advertise<nav_msgs::OccupancyGrid>("e_map", 1,true);
        pub2 = nh_.advertise<nav_msgs::OccupancyGrid>("c_map", 1,true);
        pub3 = nh_.advertise<nav_msgs::OccupancyGrid>("es_map", 1,true);
        pub4 = nh_.advertise<nav_msgs::OccupancyGrid>("r_map", 1,true);
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


        if(debug){
            cv::namedWindow(M_WINDOW);
            cv::namedWindow(E_WINDOW);
            cv::namedWindow(C_WINDOW);
            cv::namedWindow(ES_WINDOW);
            cv::namedWindow(R_WINDOW);
            cv::namedWindow(L_WINDOW);
            cv::namedWindow(A_WINDOW);
            cv::namedWindow(D_WINDOW);
            cv::namedWindow(V_WINDOW);
            cv::namedWindow(G_WINDOW);
        }
    }

    ~Reach_transf()
    {
        if(debug){
           cv::destroyWindow(M_WINDOW);
           cv::destroyWindow(E_WINDOW);
           cv::destroyWindow(C_WINDOW);
           cv::destroyWindow(ES_WINDOW);
           cv::destroyWindow(R_WINDOW);
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
    void spin(void);


    void transf_pos(void);


    bool getTreated(void){return treated;}


};


//std::vector<std::vector<signed char> > Reach_transf::close (std::vector<std::vector<signed char> > map,float radius)
//{
//    //std::vector<std::vector<signed char> >
//            n_map=std::vector<std::vector<signed char> >(map);

//    int rad=ceil(radius);

//    for(int i=0;i<map.size();i++){
//        for(int j=0;j<map[i].size();j++){

//            for (int k=-rad;k<=rad;k++){
//                 for (int l=-rad;l<=rad;l++){

//                     if( (map[std::min(std::max(i+k,0),(int)map.size()-1)][std::min(std::max(j+l,0),(int)map[i].size()-1)]==1) && (k*k+l*l<=radius*radius) )
//                        n_map[i][j]=1;

//                }
//            }
//        }
//    }

//    //std::vector<std::vector<signed char> > f_map(n_map);
//    f_map=std::vector<std::vector<signed char> >(n_map);

//    for(int i=0;i<n_map.size();i++){
//        for(int j=0;j<n_map[i].size();j++){

//            for (int k=-rad;k<=rad;k++){
//                 for (int l=-rad;l<=rad;l++){

//                     if( (n_map[std::min(std::max(i+k,0),(int)n_map.size()-1)][std::min(std::max(j+l,0),(int)n_map[i].size()-1)]!=1) && (k*k+l*l<=radius*radius) )
//                     {
//                         if (map[i][j]==-1)
//                             f_map[i][j]=-1;
//                         else
//                             f_map[i][j]=0;
//                     }

//                }
//            }
//        }
//    }

//    fr_map=std::vector<std::vector<signed char> >(f_map);

//    std::vector<std::vector<char> > labels(0);

//    for(int i=0;i<fr_map.size();i++){
//        for(int j=0;j<fr_map[i].size();j++){

//            if(fr_map[i][j]!=1) {

//                for (int k=-1;k<=0;k++){
//                     for (int l=-1;l<=-2*k-1;l++){

//                         if( (i+k)>=0 && (i+k)<fr_map.size() && (j+l)>=0 && (j+l)<fr_map[i].size() )
//                         {

//                             if( (fr_map[i+k][j+l]>1))
//                             {
//                                 if (fr_map[i][j]<1)
//                                     fr_map[i][j]=fr_map[i+k][j+l];
//                                 else if (fr_map[i+k][j+l]!=fr_map[i][j])
//                                 {
//                                     //f_map[i][j]=0;
//                                 }
//                             }
//                         }
//                    }
//                }

//                if (fr_map[i][j]<1)
//                {
//                    fr_map[i][j]=2+labels.size();
//                    //labels
//                }

//            }
//        }
//    }


//    return fr_map;
//}

void Reach_transf::show(void)
{
    //boost::mutex::scoped_lock lock(mux);
    if(count>0){
        cv::imshow(M_WINDOW,map_or);
        cv::imshow(E_WINDOW,map_erosionOp);
        cv::imshow(C_WINDOW,map_closeOp);
        cv::imshow(ES_WINDOW,map_eroded_skel);
        cv::imshow(R_WINDOW,map_reach);
        if(pos_rcv)
        {
            cv::imshow(L_WINDOW,map_label);
            cv::imshow(A_WINDOW,map_act);
            cv::imshow(V_WINDOW,map_vis);
            cv::imshow(D_WINDOW,map_debug);
            cv::imshow(G_WINDOW,map_truth);
        }
        cv::waitKey(3);
    }
}


void thinningIteration(cv::Mat& im, int iter)
{
    cv::Mat marker = cv::Mat::zeros(im.size(), CV_8UC1);

    for (int i = 1; i < im.rows-1; i++)
    {
        for (int j = 1; j < im.cols-1; j++)
        {
            uchar p2 = im.at<uchar>(i-1, j);
            uchar p3 = im.at<uchar>(i-1, j+1);
            uchar p4 = im.at<uchar>(i, j+1);
            uchar p5 = im.at<uchar>(i+1, j+1);
            uchar p6 = im.at<uchar>(i+1, j);
            uchar p7 = im.at<uchar>(i+1, j-1);
            uchar p8 = im.at<uchar>(i, j-1);
            uchar p9 = im.at<uchar>(i-1, j-1);

            int A  = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) +
                     (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) +
                     (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                     (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
            int B  = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
            int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

            if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                marker.at<uchar>(i,j) = 1;
        }
    }

    im &= ~marker;
}


cv::Mat Reach_transf::skel3(cv::Mat img)
{
    bitwise_not(img,img);
    img /= 255;

    cv::Mat prev = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::Mat diff;

    do {
        thinningIteration(img, 0);
        thinningIteration(img, 1);
        cv::absdiff(img, prev, diff);
        img.copyTo(prev);
    }
    while (cv::countNonZero(diff) > 0);

    img *= 255;
    bitwise_not(img,img);

    return img;
}

void ThinSubiteration1(cv::Mat & pSrc, cv::Mat & pDst) {
    int rows = pSrc.rows;
    int cols = pSrc.cols;
    pSrc.copyTo(pDst);
    for(int i = 0; i < rows; i++) {
            for(int j = 0; j < cols; j++) {
                    if(pSrc.at<float>(i, j) == 1.0f) {
                            /// get 8 neighbors
                            /// calculate C(p)
                            int neighbor0 = (int) pSrc.at<float>( i-1, j-1);
                            int neighbor1 = (int) pSrc.at<float>( i-1, j);
                            int neighbor2 = (int) pSrc.at<float>( i-1, j+1);
                            int neighbor3 = (int) pSrc.at<float>( i, j+1);
                            int neighbor4 = (int) pSrc.at<float>( i+1, j+1);
                            int neighbor5 = (int) pSrc.at<float>( i+1, j);
                            int neighbor6 = (int) pSrc.at<float>( i+1, j-1);
                            int neighbor7 = (int) pSrc.at<float>( i, j-1);
                            int C = int(~neighbor1 & ( neighbor2 | neighbor3)) +
                                             int(~neighbor3 & ( neighbor4 | neighbor5)) +
                                             int(~neighbor5 & ( neighbor6 | neighbor7)) +
                                             int(~neighbor7 & ( neighbor0 | neighbor1));
                            if(C == 1) {
                                    /// calculate N
                                    int N1 = int(neighbor0 | neighbor1) +
                                                     int(neighbor2 | neighbor3) +
                                                     int(neighbor4 | neighbor5) +
                                                     int(neighbor6 | neighbor7);
                                    int N2 = int(neighbor1 | neighbor2) +
                                                     int(neighbor3 | neighbor4) +
                                                     int(neighbor5 | neighbor6) +
                                                     int(neighbor7 | neighbor0);
                                    int N = cv::min(N1,N2);
                                    if ((N == 2) || (N == 3)) {
                                            /// calculate criteria 3
                                            int c3 = ( neighbor1 | neighbor2 | ~neighbor4) & neighbor3;
                                            if(c3 == 0) {
                                                    pDst.at<float>( i, j) = 0.0f;
                                            }
                                    }
                            }
                    }
            }
    }
}


void ThinSubiteration2(cv::Mat & pSrc, cv::Mat & pDst) {
    int rows = pSrc.rows;
    int cols = pSrc.cols;
    pSrc.copyTo( pDst);
    for(int i = 0; i < rows; i++) {
            for(int j = 0; j < cols; j++) {
                    if (pSrc.at<float>( i, j) == 1.0f) {
                            /// get 8 neighbors
                            /// calculate C(p)
                        int neighbor0 = (int) pSrc.at<float>( i-1, j-1);
                        int neighbor1 = (int) pSrc.at<float>( i-1, j);
                        int neighbor2 = (int) pSrc.at<float>( i-1, j+1);
                        int neighbor3 = (int) pSrc.at<float>( i, j+1);
                        int neighbor4 = (int) pSrc.at<float>( i+1, j+1);
                        int neighbor5 = (int) pSrc.at<float>( i+1, j);
                        int neighbor6 = (int) pSrc.at<float>( i+1, j-1);
                        int neighbor7 = (int) pSrc.at<float>( i, j-1);
                            int C = int(~neighbor1 & ( neighbor2 | neighbor3)) +
                                    int(~neighbor3 & ( neighbor4 | neighbor5)) +
                                    int(~neighbor5 & ( neighbor6 | neighbor7)) +
                                    int(~neighbor7 & ( neighbor0 | neighbor1));
                            if(C == 1) {
                                    /// calculate N
                                    int N1 = int(neighbor0 | neighbor1) +
                                            int(neighbor2 | neighbor3) +
                                            int(neighbor4 | neighbor5) +
                                            int(neighbor6 | neighbor7);
                                    int N2 = int(neighbor1 | neighbor2) +
                                            int(neighbor3 | neighbor4) +
                                            int(neighbor5 | neighbor6) +
                                            int(neighbor7 | neighbor0);
                                    int N = cv::min(N1,N2);
                                    if((N == 2) || (N == 3)) {
                                            int E = (neighbor5 | neighbor6 | ~neighbor0) & neighbor7;
                                            if(E == 0) {
                                                    pDst.at<float>(i, j) = 0.0f;
                                            }
                                    }
                            }
                    }
            }
    }
}


cv::Mat Reach_transf::skel2(cv::Mat img){
    bool bDone = false;

    bitwise_not(img,img);

    int rows = img.rows;
    int cols = img.cols;

    /// start to thin
    cv::Mat p_thinMat1 = cv::Mat::zeros(rows + 2, cols + 2, CV_32FC1);
    cv::Mat p_thinMat2 = cv::Mat::zeros(rows + 2, cols + 2, CV_32FC1);
    cv::Mat p_cmp = cv::Mat::zeros(rows + 2, cols + 2, CV_8UC1);

    while (bDone != true) {
            /// sub-iteration 1
            ThinSubiteration1(img, p_thinMat1);
            /// sub-iteration 2
            ThinSubiteration2(p_thinMat1, p_thinMat2);
            /// compare
            cv::compare(img, p_thinMat2, p_cmp, CV_CMP_EQ);
            /// check
            int num_non_zero = countNonZero(p_cmp);
            if(num_non_zero == (rows + 2) * (cols + 2)) {
                    bDone = true;
            }
            /// copy
            p_thinMat2.copyTo(img);
    }

    bitwise_not(img,img);

    return img;
}

cv::Mat Reach_transf::skel(cv::Mat img)
{
    bitwise_not(img,img);

    cv::threshold(img, img, 127, 255, cv::THRESH_BINARY);
    cv::Mat skel(img.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat temp;
    cv::Mat eroded;

    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    bool done;
    do
    {
      cv::erode(img, eroded, element);
      cv::dilate(eroded, temp, element); // temp = open(img)
      cv::subtract(img, temp, temp);
      cv::bitwise_or(skel, temp, skel);
      eroded.copyTo(img);

      done = (cv::countNonZero(img) == 0);
    } while (!done);

    bitwise_not(skel,skel);

    return skel;
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

        switch (kernel) {
        case 1:
            erode( or_map, er_map, element1);
            dilate( er_map, cl_map, element11 );
            break;
        case 3:
            erode( or_map, er_map, element5);
            dilate( er_map, cl_map, element15 );
            break;
        default:
            erode( or_map, er_map, element6);
            dilate( er_map, cl_map, element16 );
            break;
        }

        //erode( or_map, er_map, element1);
        //erode( cv_map, map_erosionOp, element );
        //dilate( er_map, cl_map, element3 );
        //dilate( er_map, cl_map, element1 );

        //cv::morphologyEx(or_map,cl_map,cv::MORPH_CLOSE, element1);

        es_map=er_map.clone();

        //lock.unlock();

        es_map=skel3(es_map);

        //lock.lock();


        bitwise_not( cl_map,r_map);
        bitwise_not(es_map,temp);
        //r_map= ( (r_map/255) | (temp/255) )*255;
        r_map= ( (r_map) | (temp) );
        bitwise_not(or_map,temp);
        r_map= ( (r_map) | (temp) );
        bitwise_not(r_map,r_map);

        //r_map= ( es_map | cl_map );

        //std::vector<std::vector<cv::Point> > labels=label(r_map.clone());

        map_or=or_map;
        map_erosionOp=er_map;
        map_closeOp=cl_map;
        map_eroded_skel=es_map;
        map_reach=r_map;


        //trans_pos();

        //=vis_map;

        //GridMap=close(GridMap,inf);
        //publish();

        //lock.unlock();


        ros::Duration diff = ros::Time::now() - t01;

        cout<<tf_pref<<" - Time for reach: "<<diff<<endl;

    //}


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

//    Sequence & operator= (const Sequence & other)
//    {
//            if (this != &other) // protect against invalid self-assignment
//            {
//                seq=other.seq;
//                rem=other.rem;
//            }
//            // by convention, always return *this
//            return *this;
//    }

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
    bool debug=false;

    if(opt_x==0 && opt_y==145)
        debug=true;

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

                    if( raytracing(map,i,j,ii,jj) )
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

            if(infl<defl)
            {
                for (int k=0;k<labels_unreach.size();k++){//2;k++){//
                    vector<cv::Point> frontiers;
                    //int min_x=vis_map.rows, min_y=vis_map.cols, max_x=-1, max_y=-1;
                    for(int j=0;j<labels_unreach[k].size();j++){

                        if ((labels_unreach[k][j].x+1)<vis_map.rows)
                            if(vis_map.at<uchar>(labels_unreach[k][j].x+1,labels_unreach[k][j].y)==255)
                            {
                                //vis_map.at<uchar>(labels_unreach[k][j].x,labels_unreach[k][j].y)=255;
                                frontiers.push_back(cv::Point(labels_unreach[k][j].x,labels_unreach[k][j].y));
    //                            if(labels_unreach[k][j].x>max_x)
    //                                max_x=labels_unreach[k][j].x;
    //                            if(labels_unreach[k][j].y>max_y)
    //                                max_y=labels_unreach[k][j].y;
    //                            if(labels_unreach[k][j].x<min_x)
    //                                min_x=labels_unreach[k][j].x;
    //                            if(labels_unreach[k][j].y<min_y)
    //                                min_y=labels_unreach[k][j].y;
                                continue;
                            }
                        if ((labels_unreach[k][j].y+1)<vis_map.cols)
                            if(vis_map.at<uchar>(labels_unreach[k][j].x,labels_unreach[k][j].y+1)==255)
                            {
                                frontiers.push_back(cv::Point(labels_unreach[k][j].x,labels_unreach[k][j].y));
    //                            if(labels_unreach[k][j].x>max_x)
    //                                max_x=labels_unreach[k][j].x;
    //                            if(labels_unreach[k][j].y>max_y)
    //                                max_y=labels_unreach[k][j].y;
    //                            if(labels_unreach[k][j].x<min_x)
    //                                min_x=labels_unreach[k][j].x;
    //                            if(labels_unreach[k][j].y<min_y)
    //                                min_y=labels_unreach[k][j].y;
                                continue;
                            }
                        if ((labels_unreach[k][j].x-1)>=0)
                            if(vis_map.at<uchar>(labels_unreach[k][j].x-1,labels_unreach[k][j].y)==255)
                            {
                                frontiers.push_back(cv::Point(labels_unreach[k][j].x,labels_unreach[k][j].y));
    //                            if(labels_unreach[k][j].x>max_x)
    //                                max_x=labels_unreach[k][j].x;
    //                            if(labels_unreach[k][j].y>max_y)
    //                                max_y=labels_unreach[k][j].y;
    //                            if(labels_unreach[k][j].x<min_x)
    //                                min_x=labels_unreach[k][j].x;
    //                            if(labels_unreach[k][j].y<min_y)
    //                                min_y=labels_unreach[k][j].y;
                                continue;
                            }
                        if ((labels_unreach[k][j].y-1)>=0)
                            if(vis_map.at<uchar>(labels_unreach[k][j].x,labels_unreach[k][j].y-1)==255)
                            {
                                frontiers.push_back(cv::Point(labels_unreach[k][j].x,labels_unreach[k][j].y));
    //                            if(labels_unreach[k][j].x>max_x)
    //                                max_x=labels_unreach[k][j].x;
    //                            if(labels_unreach[k][j].y>max_y)
    //                                max_y=labels_unreach[k][j].y;
    //                            if(labels_unreach[k][j].x<min_x)
    //                                min_x=labels_unreach[k][j].x;
    //                            if(labels_unreach[k][j].y<min_y)
    //                                min_y=labels_unreach[k][j].y;
                                continue;
                            }
                        //unreach_map.at<uchar>(labels_unreach[k][j].x,labels_unreach[k][j].y)=255;


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

                            //unreach_map.at<uchar>(opt_x,opt_y)=0;

                            //cout<<endl<<frontier.size()<<endl;

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

                                                            //if((angles[aa+1]-angles[aa])>PI)
                                                            //{
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

                                                            //}


            //                                                    bool found=false;
            //                                                    if((angles[aa+1]-angles[aa])>PI)
            //                                                    {
            //                                                        obt_angle=aa;
            //                                                        found=true;
            //                                                        break;
            //                                                    }
            //                                                    if(!found)
            //                                                        obt_angle=angles.size()-1;
                                                        }

                                                        //if(!found)
                                                        //    obt_angle=angles.size()-1;

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

                                                            //if((angles[aa+1]-angles[aa])>PI)
                                                            //{
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

                                                            //}


            //                                                    bool found=false;
            //                                                    if((angles[aa+1]-angles[aa])>PI)
            //                                                    {
            //                                                        obt_angle=aa;
            //                                                        found=true;
            //                                                        break;
            //                                                    }
            //                                                    if(!found)
            //                                                        obt_angle=angles.size()-1;
                                                        }

                                                        //if(!found)
                                                        //    obt_angle=angles.size()-1;

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

                                                        //if((angles[aa+1]-angles[aa])>PI)
                                                        //{
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

                                                        //}


        //                                                    bool found=false;
        //                                                    if((angles[aa+1]-angles[aa])>PI)
        //                                                    {
        //                                                        obt_angle=aa;
        //                                                        found=true;
        //                                                        break;
        //                                                    }
        //                                                    if(!found)
        //                                                        obt_angle=angles.size()-1;
                                                    }

                                                    //if(!found)
                                                    //    obt_angle=angles.size()-1;

                                                    if( ((angles[0]+PI)+(PI-angles[angles.size()-1]))>anglediff )
                                                    {
                                                        obt_angle=angles.size()-1;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }

    //                            for(int jp=0;jp<angles.size();jp++)
    //                            {
    //                                cout<<angles[jp]<<" ";
    //                            }
    //                            cout<<endl;
    //                            cout<<"Obt_angle: "<<obt_angle<<endl;
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
                                        //float angle=atan2(coly-opt_y,rowx-opt_x);
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

            //                for(int jp=0;jp<angles.size();jp++)
            //                {
            //                    cout<<angles[jp]<<" ";
            //                }
            //                cout<<endl;
            //                cout<<"Obt_angle: "<<obt_angle<<endl;


                            //cout<<"Extremes: "<<extremes[0]<<" "<<extremes[1]<<endl;
                            //cout<<"Obt angle: "<<obt_angle<<endl;

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
                                                            //map_closeOp.at<uchar>(rowx,coly)=0;
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
                                                            //map_closeOp.at<uchar>(rowx,coly)=0;
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

//                            std::vector<cv::Point> points_vis=label_seed(vis_map_temp.clone()/255,4,cv::Point(frontier[ff][0].x,frontier[ff][0].y));

//                            vis_map_temp=cv::Mat::zeros(regions.rows, regions.cols, CV_8UC1)*255;

//                            for(int pv=0;pv<points_vis.size();pv++)
//                            {
//                                vis_map_temp.at<uchar>(points_vis[pv].x,points_vis[pv].y)=255;
//                            }

//                            for(int rowx=max((opt_x-defl),0);rowx<=min((opt_x+defl),regions.rows-1);rowx++)
//                            {
//                                for(int coly=max((opt_y-defl),0);coly<=min((opt_y+defl),regions.cols-1);coly++)
//                                {
//                                    float angle=atan2(coly-opt_y,rowx-opt_x);
//                                    float dist=(rowx-opt_x)*(rowx-opt_x)+(coly-opt_y)*(coly-opt_y);

//                                    if(obt_angle==1)
//                                    {
//                                        if (angle<extremes[1] && angle>extremes[0] && (dist<=(1*defl*defl)) && (map_or.at<uchar>(rowx,coly)==0) )
//                                        {
//                                            bool stop=false;
//                                            for(int vx=-1;vx<=1;vx++)
//                                            {
//                                                for(int vy=-1;vy<=1;vy++)
//                                                {
//                                                    if( (rowx+vx)>=0 && (rowx+vx)<regions.rows && (coly+vy)>=0 && (coly+vy)<regions.cols )
//                                                    {
//                                                        if( vis_map_temp.at<uchar>(rowx+vx,coly+vy)==255  )
//                                                        {
//                                                            occ.push_back(cv::Point(rowx,coly));
//                                                            //map_closeOp.at<uchar>(rowx,coly)=0;
//                                                            stop=true;
//                                                            break;
//                                                        }
//                                                    }
//                                                }
//                                                if(stop)
//                                                    break;
//                                            }
//                                        }
//                                    }
//                                    else
//                                    {
//                                        if ( (angle<extremes[0] || angle>extremes[1]) && (dist<=(1*defl*defl)) && (map_or.at<uchar>(rowx,coly)==0) )
//                                        {
//                                            bool stop=false;
//                                            for(int vx=-1;vx<=1;vx++)
//                                            {
//                                                for(int vy=-1;vy<=1;vy++)
//                                                {
//                                                    if( (rowx+vx)>=0 && (rowx+vx)<regions.rows && (coly+vy)>=0 && (coly+vy)<regions.cols )
//                                                    {
//                                                        if( vis_map_temp.at<uchar>(rowx+vx,coly+vy)==255  )
//                                                        {
//                                                            occ.push_back(cv::Point(rowx,coly));
//                                                            //map_closeOp.at<uchar>(rowx,coly)=0;
//                                                            stop=true;
//                                                            break;
//                                                        }
//                                                    }
//                                                }
//                                                if(stop)
//                                                    break;
//                                            }
//                                        }

//                                    }

//                                }
//                            }


                            vector<vector<cv::Point> > occ_clust=cluster_points(occ);

                            vector<cv::Point> occ_critP;

                            //cout<<"Numbe of obstacles; "<<occ_clust.size()<<endl;

                            contours = cv::Mat::ones(regions.rows, regions.cols, CV_8UC1)*255;

                            for(int ind=0;ind<occ_clust.size();ind++)
                            {

                                for(int occ_p=0;occ_p<occ_clust[ind].size();occ_p++)
                                {
                                    contours.at<uchar>(occ_clust[ind][occ_p].x,occ_clust[ind][occ_p].y)=0;
                                }


                                //cout<<"Size of obstacle "<<ind+1<<": "<<occ_clust[ind].size()<<endl;



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

//                                            for(int vx=-1;vx<=1;vx++)
//                                            {
//                                                for(int vy=-1;vy<=1;vy++)
//                                                {
                                                    //if( (occ_clust[ind][occ_p].x+vx)>=0 && (occ_clust[ind][occ_p].x+vx)<regions.rows && (occ_clust[ind][occ_p].y+vy)>=0 && (occ_clust[ind][occ_p].x+vy)<regions.cols )
                                                    //{
                                                        //if( (abs(vx)+abs(vy))==1 && vis_map_temp.at<uchar>(occ_clust[ind][occ_p].x+vx,occ_clust[ind][occ_p].y+vy)==255  )
                                            if( contours_check.at<uchar>(occ_clust[ind][occ_p].x,occ_clust[ind][occ_p].y)==0  )
                                            {
                                                pos_x=occ_clust[ind][occ_p].x;
                                                pos_y=occ_clust[ind][occ_p].y;
                                                stop=true;
                                                break;
                                            }
                                                    //}
//                                                }
//                                                if(stop)
//                                                    break;
//                                            }
//                                            if(stop)
//                                                break;
                                        }

                                        if(stop)
                                        {
//                                            cout<<pos_x<<" "<<pos_y<<endl;
//                                            cout<<(contours.at<uchar>(pos_x-1, pos_y-1)==0)<<" "<<(contours.at<uchar>(pos_x-1, pos_y)==0)<<" "<<(contours.at<uchar>(pos_x-1, pos_y+1)==0)<<endl;
//                                            cout<<(contours.at<uchar>(pos_x, pos_y-1)==0)<<" "<<(contours.at<uchar>(pos_x, pos_y)==0)<<" "<<(contours.at<uchar>(pos_x, pos_y+1)==0)<<endl;
//                                            cout<<(contours.at<uchar>(pos_x+1, pos_y-1)==0)<<" "<<(contours.at<uchar>(pos_x+1, pos_y)==0)<<" "<<(contours.at<uchar>(pos_x+1, pos_y+1)==0)<<endl;
//                                            cout<<(contours.at<uchar>(pos_x+2, pos_y-1)==0)<<" "<<(contours.at<uchar>(pos_x+2, pos_y)==0)<<" "<<(contours.at<uchar>(pos_x+2, pos_y+1)==0)<<endl;
//                                            cout<<endl;


                                            const int dir=8; // number of possible directions to go at any position
                                            //if dir==4
                                            //static int dx[dir]={1, 0, -1, 0};
                                            //static int dy[dir]={0, 1, 0, -1};
                                            //if dir==8
                                            int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
                                            int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};

                                            vector<Chain> chain;
                                            chain.clear();

                                            int prev_d;

                                            bool cont_cond=true;

                                            int sign=1;

                                            while(cont_cond)
                                            {
                                                //cout<<"Stuck 111!!!!"<<endl;

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

                                                                //cout<<c.x<<" "<<c.y<<" "<<c.d<<endl;


                                                                if(chain[0]==c)
                                                                {
                                                                    cont_cond=false;
                                                                }
    //                                                            if(chain[0].x==c.x && chain[0].y==c.y)
    //                                                                cout<<"Almost finished: "<<chain[0].d<<" "<<c.d<<endl;

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
                                                            if(  contours.at<uchar>(pos_x+dx[d],pos_y+dy[d])==0 )//&& (abs(dx[d])+abs(dy[d]))==1 )
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
                                                                            //break;
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
                                                                               //break;
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

    //                                                if(valid_rotation)
    //                                                    cout<<pos_x<<" "<<pos_y<<endl;
    //                                                else
    //                                                    cout<<"No point"<<endl;
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

                                                //cout<<chain[c].x<<" "<<chain[c].y<<" "<<chain[c].d<<endl;
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

                                                //cout<<diffs[c]<<" ";
                                            }
                                             //cout<<endl;

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
                                //cout<<occ_critP[c].x<<"; "<<occ_critP[c].y<<endl;
                                if(contours_filt.at<uchar>(occ_critP[c].x,occ_critP[c].y)==255)
                                    continue;
                                else
                                {
                                    //cout<<occ_critP[c].x<<"; "<<occ_critP[c].y<<endl;

//                                    bool stop=false;
//                                    for(int vx=-1;vx<=1;vx++)
//                                    {
//                                        for(int vy=-1;vy<=1;vy++)
//                                        {
//                                            if( (occ_critP[c].x+vx)>=0 && (occ_critP[c].x+vx)<regions.rows && (occ_critP[c].y+vy)>=0 && (occ_critP[c].y+vy)<regions.cols )
//                                            {
//                                                if( vis_map_temp.at<uchar>(occ_critP[c].x+vx,occ_critP[c].y+vy)==0 && (contours.at<uchar>(occ_critP[c].x+vx,occ_critP[c].y+vy)==255)  )
//                                                {
//                                                    stop=true;
//                                                    break;
//                                                }
//                                            }
//                                        }
//                                        if(stop)
//                                            break;
//                                    }
//                                    if(stop)
//                                        continue;

                                    contours_filt.at<uchar>(occ_critP[c].x,occ_critP[c].y)=255;
                                    occ_crit_filt.push_back(cv::Point(occ_critP[c].x,occ_critP[c].y));
                                    //cout<<occ_critP[c].x<<"; "<<occ_critP[c].y<<endl;

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


                                //p_x=sign_x*((int)round(sign_x*(tempx+temp*cos_ang)+p_x)-p_x);
                                //p_y=sign_y*((int)round(sign_y*(tempy+temp*sin_ang)+p_y)-p_y);

//                                bool count_free=false;

                                //cout<<p_x<<" "<<p_y<<" "<<sign_x*(tempx+temp*cos_ang)<<" "<<sign_y*(tempy+temp*sin_ang)<<" "<<tempx<<" "<<tempy<<" "<<dist+temp<<" "<<dist<<endl;

                                int n=1;

                                while( (dist+temp)<=defl && p_x>=0 && p_x<regions.rows && p_y>=0 && p_y<regions.cols )
                                {
                                    n++;
                                    //cout<<"Stuck 222!!!!"<<endl;

//                                    if(!count_free)
//                                    {
//                                        if(vis_map_temp.at<uchar>(p_x,p_y)==255)
//                                            count_free=true;
//                                    }
//                                    else
//                                        if(vis_map_temp.at<uchar>(p_x,p_y)==0)
//                                            break;


                                    //if(occ_crit_filt[c].x==15 && occ_crit_filt[c].y==81)
                                    //if(n<50)
                                        //cout<<p_x<<" "<<p_y<<" "<<sign_x*(tempx+temp*cos_ang)<<" "<<sign_y*(tempy+temp*sin_ang)<<" "<<tempx<<" "<<tempy<<" "<<dist+temp<<" "<<dist<<endl;

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


                                    //if(n<50)
                                        //cout<<p_x<<" "<<p_y<<" "<<sign_x*(tempx+temp*cos_ang)+p_x<<" "<<sign_y*(tempy+temp*sin_ang)+p_y<<" "<<tempx<<" "<<tempy<<" "<<dist+temp<<" "<<dist<<endl;


                                    if(sign_x==0)
                                        p_x=p_x;
                                    else
                                        p_x=sign_x*((int)round(sign_x*(tempx+temp*cos_ang)+p_x)-p_x);

                                    if(sign_y==0)
                                        p_y=p_y;
                                    else
                                        p_y=sign_y*((int)round(sign_y*(tempy+temp*sin_ang)+p_y)-p_y);

                                    //p_x=sign_x*((int)round(sign_x*(tempx+temp*cos_ang)+p_x)-p_x);
                                    //p_y=sign_y*((int)round(sign_y*(tempy+temp*sin_ang)+p_y)-p_y);




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


                            //cout<<"Critical points: "<<occ_critP.size()<<"; After filtering: "<<occ_crit_filt.size()<<endl;

//                            for(int c=0;c<occ_crit_filt.size();c++)
//                            {
//                                cout<<occ_crit_filt[c].x<<"; "<<occ_crit_filt[c].y<<endl;
//                            }

//                            map_closeOp=contours_filt;

//                            int label_vis=255;

//                            for(int pv=0;pv<pre_vis.size();pv++)
//                            {
//                                if( vis_map_temp.at<uchar>(pre_vis[pv].x,pre_vis[pv].y)==label_vis )
//                                {
//                                    vis_map.at<uchar>(pre_vis[pv].x,pre_vis[pv].y)=255;
//                                }
//                            }

                            act_map.at<uchar>(opt_x,opt_y)=0;

                            //cout<<extremes[0]<<" "<<extremes[1]<<" "<<obt_angle<<" "<<endl;



                        }
                    }
                }
            }

            //vis_map=vis_map_temp;

            //=unreach_map;

            map_label=l_map;
            map_act=act_map;
            map_vis=vis_map;

            map_debug=unreach_map;
            //map_debug=contours;


            ros::Duration diff = ros::Time::now() - t01;

            cout<<tf_pref<<" - Time for label: "<<diff<<endl;


            ros::Time t2=ros::Time::now();

            map_truth=brute_force(map_or, map_label, defl);


            diff = ros::Time::now() - t2;

            cout<<tf_pref<<" - Time for brute force: "<<diff<<endl;

            map_truth.at<uchar>(pos_x,pos_y)=0;
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

        n_msg=Mat2RosMsg(map_eroded_skel, msg_rcv_pub);
        pub3.publish(n_msg);

        n_msg=Mat2RosMsg(map_reach , msg_rcv_pub);
        pub4.publish(n_msg);

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

    reach.transf_pos();

    if(debug)
        reach.show();

    reach.publish();

    //reach.publish();

    loop_rate.sleep();
  }


  return 0;
}
