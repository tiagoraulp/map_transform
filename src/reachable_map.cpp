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
    
    ros::Subscriber sub;

    tf::TransformListener pos_listener;

    std::vector<std::vector<cv::Point> >  label(const cv::Mat binary);
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

    int kernel;

    string tf_pref;

    int height, width;

    float res, or_x, or_y;

    nav_msgs::OccupancyGrid msg_rcv,msg_rcv_pub;

    cv::Mat cv_map, map_or, map_erosionOp, map_closeOp, map_eroded_skel, map_reach, map_label;

public:

    Reach_transf(ros::NodeHandle nh): nh_(nh)
    {

        pub = nh_.advertise<nav_msgs::OccupancyGrid>("e_map", 1,true);
        pub2 = nh_.advertise<nav_msgs::OccupancyGrid>("c_map", 1,true);
        pub3 = nh_.advertise<nav_msgs::OccupancyGrid>("es_map", 1,true);
        pub4 = nh_.advertise<nav_msgs::OccupancyGrid>("r_map", 1,true);
        pub5 = nh_.advertise<nav_msgs::OccupancyGrid>("l_map", 1,true);

        sub = nh_.subscribe("map", 1, &Reach_transf::rcv_map, this);

        nh_.param("infl", infl, 5);

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
        if(pos_rcv) cv::imshow(L_WINDOW,map_label);
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

    cv_map = cv::Mat(msg->info.width, msg->info.height, CV_8UC1);


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

                mapDataIterC++;
            }
    }

    ++count;

    msg_rcv=*msg;

    treated=false;

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



    if (count>0 && !treated)
    {

        ros::Time t01=ros::Time::now();

        treated=true;
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


        cv::Mat or_map, er_map, cl_map, es_map, r_map, temp;

        or_map=cv_map.clone();
        msg_rcv_pub=msg_rcv;


        /// Apply the erosion operation
        ///
        ///

        switch (kernel) {
        case 1:
            erode( or_map, er_map, element1);
            dilate( er_map, cl_map, element1 );
            break;
        case 3:
            erode( or_map, er_map, element5);
            dilate( er_map, cl_map, element5 );
            break;
        default:
            erode( or_map, er_map, element6);
            dilate( er_map, cl_map, element6 );
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

        //map_label=l_map;

        //GridMap=close(GridMap,inf);
        //publish();

        //lock.unlock();


        ros::Duration diff = ros::Time::now() - t01;

        cout<<tf_pref<<" - Time for reach: "<<diff<<endl;

    }


}

void Reach_transf::transf_pos(void)
{





    tf::StampedTransform transform;
    try{
        pos_listener.lookupTransform("/map", tf_pref+"/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_INFO("%s",ex.what());
      return ;
    }



    if(count>0)
    {
        ros::Time t01=ros::Time::now();

        int pos_x=(int) round((transform.getOrigin().x()-or_x)/res);

        int pos_y=(int) round((transform.getOrigin().y()-or_y)/res);

        cv::Mat temp_labelling, temp;
        //bitwise_not( map_erosionOp.clone() , temp_labelling);

        std::vector<std::vector<cv::Point> > labels=label(map_erosionOp.clone()/255);


        int label_pos=-1;


        for (int i=0;i<labels.size();i++){
            for(int j=0;j<labels[i].size();j++){
                //ROS_INFO("%d-%d ; %d-%d",i+1, labels.size(),j+1,labels[i].size());

                //ROS_INFO("%d %d",labels[i][j].x, labels[i][j].y);
                if( pos_x==labels[i][j].x && pos_y==labels[i][j].y){

                    label_pos=i;
                    break;

                }
            }

            if(label_pos>-1)
                break;
        }



        cv::Mat l_map=map_erosionOp.clone(), unreach_map, regions;


        for (int i=0;i<labels.size();i++){
            for(int j=0;j<labels[i].size();j++){
                if( i!=label_pos ){

                    l_map.at<uchar>(labels[i][j].x,labels[i][j].y)=0;

                }
            }
         }


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




        /// Apply the erosion operation
        ///
        ///

        switch (kernel) {
        case 1:
            //erode( or_map, er_map, element1);
            dilate( l_map, l_map, element1 );
            break;
        case 3:
            //erode( or_map, er_map, element5);
            dilate( l_map, l_map,  element5 );
            break;
        default:
            //erode( or_map, er_map, element6);
            dilate( l_map, l_map,  element6 );
            break;
        }


        bitwise_not(map_or,temp);

        unreach_map=l_map|temp;

        bitwise_not( unreach_map.clone() , temp_labelling);

        std::vector<std::vector<cv::Point> > labels_unreach=label(temp_labelling/255);


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

        for (int k=0;k<labels_unreach.size();k++){//4;k++){//
            vector<cv::Point> frontier;
            int min_x=l_map.rows, min_y=l_map.cols, max_x=-1, max_y=-1;
            for(int j=0;j<labels_unreach[k].size();j++){

                if ((labels_unreach[k][j].x+1)<l_map.rows)
                    if(l_map.at<uchar>(labels_unreach[k][j].x+1,labels_unreach[k][j].y)==255)
                    {
                        //l_map.at<uchar>(labels_unreach[k][j].x,labels_unreach[k][j].y)=255;
                        frontier.push_back(cv::Point(labels_unreach[k][j].x,labels_unreach[k][j].y));
                        if(labels_unreach[k][j].x>max_x)
                            max_x=labels_unreach[k][j].x;
                        if(labels_unreach[k][j].y>max_y)
                            max_y=labels_unreach[k][j].y;
                        if(labels_unreach[k][j].x<min_x)
                            min_x=labels_unreach[k][j].x;
                        if(labels_unreach[k][j].y<min_y)
                            min_y=labels_unreach[k][j].y;
                        continue;
                    }
                if ((labels_unreach[k][j].y+1)<l_map.cols)
                    if(l_map.at<uchar>(labels_unreach[k][j].x,labels_unreach[k][j].y+1)==255)
                    {
                        frontier.push_back(cv::Point(labels_unreach[k][j].x,labels_unreach[k][j].y));
                        if(labels_unreach[k][j].x>max_x)
                            max_x=labels_unreach[k][j].x;
                        if(labels_unreach[k][j].y>max_y)
                            max_y=labels_unreach[k][j].y;
                        if(labels_unreach[k][j].x<min_x)
                            min_x=labels_unreach[k][j].x;
                        if(labels_unreach[k][j].y<min_y)
                            min_y=labels_unreach[k][j].y;
                        continue;
                    }
                if ((labels_unreach[k][j].x-1)>=0)
                    if(l_map.at<uchar>(labels_unreach[k][j].x-1,labels_unreach[k][j].y)==255)
                    {
                        frontier.push_back(cv::Point(labels_unreach[k][j].x,labels_unreach[k][j].y));
                        if(labels_unreach[k][j].x>max_x)
                            max_x=labels_unreach[k][j].x;
                        if(labels_unreach[k][j].y>max_y)
                            max_y=labels_unreach[k][j].y;
                        if(labels_unreach[k][j].x<min_x)
                            min_x=labels_unreach[k][j].x;
                        if(labels_unreach[k][j].y<min_y)
                            min_y=labels_unreach[k][j].y;
                        continue;
                    }
                if ((labels_unreach[k][j].y-1)>=0)
                    if(l_map.at<uchar>(labels_unreach[k][j].x,labels_unreach[k][j].y-1)==255)
                    {
                        frontier.push_back(cv::Point(labels_unreach[k][j].x,labels_unreach[k][j].y));
                        if(labels_unreach[k][j].x>max_x)
                            max_x=labels_unreach[k][j].x;
                        if(labels_unreach[k][j].y>max_y)
                            max_y=labels_unreach[k][j].y;
                        if(labels_unreach[k][j].x<min_x)
                            min_x=labels_unreach[k][j].x;
                        if(labels_unreach[k][j].y<min_y)
                            min_y=labels_unreach[k][j].y;
                        continue;
                    }
                //unreach_map.at<uchar>(labels_unreach[k][j].x,labels_unreach[k][j].y)=255;


            }

            if(frontier.size()>0)
            {

                double min_sum=-1; int opt_x, opt_y;

                for(int x=max(min_x-infl,0);x<min(max_x+infl,l_map.rows);x++)
                {
                    for(int y=max(min_y-infl,0);y<min(max_y+infl,l_map.cols);y++)
                    {
                        if(l_map.at<uchar>(x,y)==255 && map_erosionOp.at<uchar>(x,y)==255)
                        {
                            double sum=0;
                            for(int l=0;l<frontier.size();l++){
                                sum+=(frontier[l].x-x)*(frontier[l].x-x)+(frontier[l].y-y)*(frontier[l].y-y);
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

                unreach_map.at<uchar>(opt_x,opt_y)=0;



                vector<float> angles;
                vector<int> angles_x;
                vector<int> angles_y;

                int obt_angle=-1;

                for(int l=0;l<frontier.size();l++)
                {
                    float angle=atan2(frontier[l].y-opt_y,frontier[l].x-opt_x);
                    vector<float>::iterator it=angles.begin();
                    vector<int>::iterator itx=angles_x.begin();
                    vector<int>::iterator ity=angles_y.begin();
                    if(angles.size()==0)
                    {
                        angles.push_back(angle);
                        angles_x.push_back(frontier[l].x);
                        angles_y.push_back(frontier[l].y);
                    }
                    else
                    {
                        for(int a=0;a<angles.size();a++)
                        {
                            if(angle<angles[a])
                            {
                                angles.insert(it,angle);
                                angles_x.insert(itx,frontier[l].x);
                                angles_y.insert(ity,frontier[l].y);


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
                                            for(int aa=0;aa<(angles.size()-1);aa++)
                                            {
                                                bool found=false;
                                                if((angles[aa+1]-angles[aa])>PI)
                                                {
                                                    obt_angle=aa;
                                                    found=true;
                                                    break;
                                                }
                                                if(!found)
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
                                            for(int aa=0;aa<(angles.size()-1);aa++)
                                            {
                                                bool found=false;
                                                if((angles[aa+1]-angles[aa])>PI)
                                                {
                                                    obt_angle=aa;
                                                    found=true;
                                                    break;
                                                }
                                                if(!found)
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
                            angles_x.insert(angles_x.end(),frontier[l].x);
                            angles_y.insert(angles_y.end(),frontier[l].y);


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
                                        for(int aa=0;aa<(angles.size()-1);aa++)
                                        {
                                            bool found=false;
                                            if((angles[aa+1]-angles[aa])>PI)
                                            {
                                                obt_angle=aa;
                                                found=true;
                                                break;
                                            }
                                            if(!found)
                                                obt_angle=angles.size()-1;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    for(int jp=0;jp<angles.size();jp++)
                    {
                        cout<<angles[jp]<<" ";
                    }
                    cout<<endl;
                    cout<<"Obt_angle: "<<obt_angle<<endl;
                }

                vector<float> extremes;extremes.clear();

                if(angles.size()==1)
                {
                    //// TODO: only one point; neighbor points
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

                //// TODO: only one point; neighbor points

                for(int rowx=max((opt_x-infl-5*infl),0);rowx<=min((opt_x+infl+5*infl),regions.rows-1);rowx++)
                {
                    for(int coly=max((opt_y-infl-5*infl),0);coly<=min((opt_y+infl+5*infl),regions.cols-1);coly++)
                    {
                        float angle=atan2(coly-opt_y,rowx-opt_x);
                        float dist=(rowx-opt_x)*(rowx-opt_x)+(coly-opt_y)*(coly-opt_y);

                        if(obt_angle==1)
                        {
                            if(angle<extremes[1] && angle>extremes[0] && (dist<=(36*infl*infl)) && regions.at<uchar>(rowx,coly)==k+2)
                            {
                                l_map.at<uchar>(rowx,coly)=255;
                            }
                        }
                        else
                        {
                            if( (angle<extremes[0] || angle>extremes[1]) && (dist<=(36*infl*infl)) && regions.at<uchar>(rowx,coly)==k+2)
                            {
                                l_map.at<uchar>(rowx,coly)=255;
                            }
                        }

                    }
                }

                l_map.at<uchar>(opt_x,opt_y)=0;

                cout<<extremes[0]<<" "<<extremes[1]<<" "<<obt_angle<<" "<<endl;



            }
        }



        //map_label=unreach_map;
        map_label=l_map;

        pos_rcv=true;


        ros::Duration diff = ros::Time::now() - t01;

        //cout<<tf_pref<<" - Time for label: "<<diff<<endl;



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
            n_msg=Mat2RosMsg(map_label , msg_rcv_pub);
            pub5.publish(n_msg);
        }


        std::stringstream ss;
        ss << tf_pref+"-> Message sent: " << count;
        //msg.data = ss.str();
        //ROS_INFO("%s", ss.str().c_str());


}


std::vector<std::vector<cv::Point> >  Reach_transf::label(const cv::Mat binary)
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
            cv::floodFill(label_image, cv::Point(x,y), cv::Scalar(label_count), &rect, cv::Scalar(0), cv::Scalar(0), 4);

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



int main(int argc, char **argv)
{
    ros::init(argc, argv, "reach");
    debug=true;
    if(argc==2)
        if(string(argv[1])==string("0"))
            debug=false;
  

  ros::NodeHandle nh("~");


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
