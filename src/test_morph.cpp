#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <map_transform/ParametersncConfig.h>

#include <sstream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <thread>         // std::thread
#include <mutex>          // std::mutex

std::mutex mtx;           // mutex for critical section




using namespace std;

const double PI = 3.141592653589793;

//bool debug;

string filename;

static const std::string M_WINDOW = "Map";
static const std::string E_WINDOW = "Erosion";
static const std::string C_WINDOW = "Close";
static const std::string L_WINDOW = "Labelled";
static const std::string A_WINDOW = "Actuation";
static const std::string V_WINDOW = "Visibility";
static const std::string D_WINDOW = "Debug";
static const std::string G_WINDOW = "Ground_Truth";
static const std::string S_WINDOW = "Structuring Element";


class Elem{
public:
    vector<cv::Mat> elems;
    cv::Point pt;
    int  pu, pb, pl, pr;
};


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

    dynamic_reconfigure::Server<map_transform::ParametersncConfig> server;
    dynamic_reconfigure::Server<map_transform::ParametersncConfig>::CallbackType func;


    tf::TransformListener pos_listener;

    std::vector<std::vector<cv::Point> >  label(const cv::Mat binary, int conn);
    std::vector<cv::Point> label_seed(const cv::Mat binary, int conn, cv::Point seed);
    nav_msgs::OccupancyGrid Mat2RosMsg(cv::Mat map ,const nav_msgs::OccupancyGrid& msg);
    //std::vector<std::vector<signed char> > close (std::vector<std::vector<signed char> > map,float radius);

    void rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    void callbackParameters(map_transform::ParametersncConfig &config, uint32_t level);


    int count;

    bool pos_rcv;

    bool treated, treated2;

    double infl;

    double defl;

    int kernel;

    cv::Mat robot;

    int prev_x, prev_y;

    string tf_pref;

    int height, width;

    float res, or_x, or_y;

    bool _debug, gt, gt_c, changed, changed2;

    double rxr,ryr, rtr, rcx, rcy, rct;

    int angle_res;

    double angle_debug;

    Elem robot_or;

    nav_msgs::OccupancyGrid msg_rcv,msg_rcv_pub;

    cv::Mat cv_map, map_or, map_erosionOp, map_closeOp, map_eroded_skel, map_reach, map_label , map_act, map_vis, map_debug, map_truth,struct_elem,map_erosionOpPrintColor;

public:

    //Reach_transf(ros::NodeHandle nh, cv::Mat rob, bool debug_): nh_(nh), robot(rob), _debug(debug_)
    Reach_transf(ros::NodeHandle nh, cv::Mat rob): nh_(nh), robot(rob)
    {



        pub = nh_.advertise<nav_msgs::OccupancyGrid>("e_map", 1,true);
        pub2 = nh_.advertise<nav_msgs::OccupancyGrid>("c_map", 1,true);
        pub5 = nh_.advertise<nav_msgs::OccupancyGrid>("l_map", 1,true);
        pub6 = nh_.advertise<nav_msgs::OccupancyGrid>("a_map", 1,true);
        pub7 = nh_.advertise<nav_msgs::OccupancyGrid>("v_map", 1,true);
        pub8 = nh_.advertise<nav_msgs::OccupancyGrid>("g_map", 1,true);

        sub = nh_.subscribe("map", 1, &Reach_transf::rcv_map, this);




        nh_.param("infl", infl, 100.0);
        infl/=100;
        nh_.param("defl", defl, 100.0);
        defl/=100;
        nh_.param("rx", rcx, 50.0);
        rcx/=100;
        nh_.param("ry", rcy, 50.0);
        rcy/=100;
        nh_.param("rt", rct, 0.0);

        nh_.param("x", rxr, 10.0);
        nh_.param("y", ryr, 10.0);
        nh_.param("theta", rtr, 0.0);

        nh_.param("ground_truth", gt, false);

        nh_.param("angle_res", angle_res, 32);

        nh_.param("debug_angle", angle_debug, 0.0);

        nh_.param("debug", _debug, true);

        nh_.param("kernel", kernel, 2);

        nh_.param("tf_prefix", tf_pref, std::string(""));



        prev_x=-1; prev_y=-1;

        count=0;

        func = boost::bind(&Reach_transf::callbackParameters, this,_1, _2);
        server.setCallback(func);




        treated=false;
        treated2=false;

        pos_rcv=false;

        //gt=false;

        gt_c=false;

        changed=false;
        changed2=false;

        //rxr=10;
        //ryr=10;
        //rtr=0;

        //rcx=0.5;
        //rcy=0.5;
        //rct=0;

        //angle_res=32;

        //angle_debug=0;




        if(_debug){
            cv::namedWindow(M_WINDOW);
            cv::namedWindow(E_WINDOW);
            cv::namedWindow(C_WINDOW);
            cv::namedWindow(S_WINDOW);
            cv::namedWindow(D_WINDOW);
            if(pos_rcv)
            {
                cv::namedWindow(L_WINDOW);
                cv::namedWindow(A_WINDOW);
                cv::namedWindow(V_WINDOW);
                if(gt && gt_c)
                    cv::namedWindow(G_WINDOW);
            }
        }


    }

    ~Reach_transf()
    {
        if(_debug){
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

    void show(void);
    void publish(void);
    void transf(void);
    void spin(void);


    void transf_pos(void);


    bool getTreated(void){return treated;}


};

void Reach_transf::callbackParameters(map_transform::ParametersncConfig &config, uint32_t level) {

    infl=config.infl/100;
    defl=config.defl/100;
    rcx=config.rx/100;
    rcy=config.ry/100;
    rct=config.rt;
    angle_res=config.angle_res;
    _debug=config.debug;
    gt=config.ground_truth;
    angle_debug=config.debug_angle;
    rxr=config.x;
    ryr=config.y;
    rtr=config.theta;
    _debug=config.debug;

    changed=true;
    changed2=true;


    //if(count>0)
    //{
    //    transf();
    //    transf_pos();
    //}

}



void Reach_transf::show(void)
{

    //boost::mutex::scoped_lock lock(mux);
    if(count>0 && _debug){


        cv::imshow(M_WINDOW,map_or);
        cv::imshow(E_WINDOW,map_erosionOpPrintColor);
        cv::imshow(C_WINDOW,map_closeOp);

        cv::imshow(S_WINDOW,struct_elem);

        cv::imshow(D_WINDOW,map_debug);


        if(pos_rcv)
        {
            cv::imshow(L_WINDOW,map_label);
            cv::imshow(A_WINDOW,map_act);
            cv::imshow(V_WINDOW,map_vis);
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


nav_msgs::OccupancyGrid Reach_transf::Mat2RosMsg(cv::Mat map ,const nav_msgs::OccupancyGrid& msg)
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

void Reach_transf::rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
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

    treated2=treated;

    //transf();

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
    cv::Mat result;

    vector<cv::Mat> channels(3);

    channels[0]=(img1&img2/255*c_b[2])+((~img1)&(~img2)/255*c_n[2])+((~img1)&img2/255*c_2[2])+(img1&(~img2)/255*c_1[2]);
    channels[1]=(img1&img2/255*c_b[1])+((~img1)&(~img2)/255*c_n[1])+((~img1)&img2/255*c_2[1])+(img1&(~img2)/255*c_1[1]);
    channels[2]=(img1&img2/255*c_b[0])+((~img1)&(~img2)/255*c_n[0])+((~img1)&img2/255*c_2[0])+(img1&(~img2)/255*c_1[0]);

    cv::merge(channels, result);

    return result;
}

cv::Mat color_print3(cv::Mat img1, cv::Mat img2, cv::Mat img3, unsigned char* c_123, unsigned char* c_12, unsigned char* c_13, unsigned char* c_23, unsigned char* c_1, unsigned char* c_2, unsigned char* c_3, unsigned char* c_0)
{
    cv::Mat result;

    vector<cv::Mat> channels(3);

    channels[0]=(img1&img2&img3/255*c_123[2])+(img1&img2&(~img3)/255*c_12[2])+(img1&(~img2)&img3/255*c_13[2])+((~img1)&img2&img3/255*c_23[2])+(img1&(~img2)&(~img3)/255*c_1[2])+((~img1)&img2&(~img3)/255*c_2[2])+((~img1)&(~img2)&img3/255*c_3[2])+((~img1)&(~img2)&(~img3)/255*c_0[2]);
    channels[1]=(img1&img2&img3/255*c_123[1])+(img1&img2&(~img3)/255*c_12[1])+(img1&(~img2)&img3/255*c_13[1])+((~img1)&img2&img3/255*c_23[1])+(img1&(~img2)&(~img3)/255*c_1[1])+((~img1)&img2&(~img3)/255*c_2[1])+((~img1)&(~img2)&img3/255*c_3[1])+((~img1)&(~img2)&(~img3)/255*c_0[1]);
    channels[2]=(img1&img2&img3/255*c_123[0])+(img1&img2&(~img3)/255*c_12[0])+(img1&(~img2)&img3/255*c_13[0])+((~img1)&img2&img3/255*c_23[0])+(img1&(~img2)&(~img3)/255*c_1[0])+((~img1)&img2&(~img3)/255*c_2[0])+((~img1)&(~img2)&img3/255*c_3[0])+((~img1)&(~img2)&(~img3)/255*c_0[0]);

    cv::merge(channels, result);

    return result;
}





Elem multiElem(cv::Mat elem, cv::Point2f pt, double orient ,double scale, int res)
{
    Elem result;

    int pl, pr, pu, pb;

    pu=elem.rows/2*max(0.0,scale-1);
    pb=elem.rows/2*max(0.0,scale-1);
    pl=elem.cols/2*max(0.0,scale-1);
    pr=elem.cols/2*max(0.0,scale-1);

    cv::copyMakeBorder(elem,elem,pu,pb,pl,pr,cv::BORDER_CONSTANT,cv::Scalar(0));

    pt.x=pt.x+pl;
    pt.y=pt.y+pu;


    int max_r=max(pt.x,elem.cols-pt.x), max_c=max(pt.y,elem.rows-pt.y);

    int max_d=(int)(sqrt(max_r*max_r+max_c*max_c));

    pl=max_d-pt.x+1;
    pr=max_d-elem.cols+pt.x+1;
    pu=max_d-pt.y+1;
    pb=max_d-elem.rows+pt.y+1;

    cv::copyMakeBorder(elem,elem,pu,pb,pl,pr,cv::BORDER_CONSTANT,cv::Scalar(0));

    pt.x=pt.x+pl;
    pt.y=pt.y+pu;


    result.pt=cv::Point(pt.x,pt.y);



    pu=pt.y+1;
    pb=elem.rows-pt.y+1;
    pl=pt.x+1;
    pr=elem.cols-pt.x+1;

    result.pl=pl;
    result.pr=pr;
    result.pb=pb;
    result.pu=pu;


    for(int i=0;i<res;i++)
    {
        cv::Mat elem_t;
        cv::Mat r = cv::getRotationMatrix2D(pt, -orient+(360.0/res*i), scale);



        cv::warpAffine(elem, elem_t, r, cv::Size(elem.rows, elem.cols),cv::INTER_LINEAR);

        result.elems.push_back(elem_t);
    }

    return result;
}

vector<cv::Mat> multiErosion(cv::Mat map, Elem robot_or)
{
    vector<cv::Mat> result;
    cv::Mat map_n=map.clone();


    for(int i=0;i<robot_or.elems.size();i++)
    {
        cv::Mat temp;
        cv::erode( map_n, temp, robot_or.elems[i], robot_or.pt,1, cv::BORDER_CONSTANT,cv::Scalar(0));

        result.push_back(temp);
    }

    return result;
}


vector<cv::Mat> multiDilation(vector<cv::Mat> map_er, Elem robot_or )
{


    vector<cv::Mat> result;

    cv::Point pr;
    pr.x=robot_or.elems[0].cols-1-robot_or.pt.x;
    pr.y=robot_or.elems[0].rows-1-robot_or.pt.y;

    for(int i=0;i<robot_or.elems.size();i++)
    {
        cv::Mat temp, elem_t;
        flip(robot_or.elems[i], elem_t, -1);


        cv::dilate(map_er[i], temp, elem_t, pr );

        result.push_back(temp);


    }

    return result;


}

void Reach_transf::transf(void)
{

    if(count>0 && ((!treated2) || changed2) )
    {
        treated2=true;
        changed=false;
        ros::Time t01=ros::Time::now();

        cv::Mat or_map, er_map, cl_map, or_mapN;

        or_map=cv_map.clone();
        msg_rcv_pub=msg_rcv;


        cv::Mat elem = robot.clone();

        cv::Point2f pt=cv::Point2f(elem.cols*rcx,elem.rows*rcy);

        robot_or=multiElem(elem, pt,rct, infl, angle_res);


        cv::copyMakeBorder(or_map,or_mapN,robot_or.pu,robot_or.pb,robot_or.pl,robot_or.pr,cv::BORDER_CONSTANT,cv::Scalar(0));



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

            //temp_color=color_print3(mer_map[i], mcl_map[i], or_map, c123, c12, c13, c23, c1, c2, c3, c0 );
            //text.str("Test ");
            //text << i;
            //cv::imshow(text.str(), temp_color);
            //cv::waitKey(3);

        }

        er_map=mer_map[m_a].clone();
        cl_map=mcl_map[m_a].clone();


        map_debug=color_print3(er_map, cl_map, or_mapN, c123, c12, c13, c23, c1, c2, c3, c0 );

        map_or=or_map;
        map_erosionOp=er_map;

        cv::Rect rec(robot_or.pl,robot_or.pu, or_map.cols, or_map.rows);
        map_closeOp=cl_map(rec);

        struct_elem=robot_or.elems[m_a]*255;


//        vector<int> compression_params;
//        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//        compression_params.push_back(9);
//        try {
//            cv::imwrite("/home/tiago/map_debug.png", map_debug, compression_params);
//        }
//        catch (exception& e)
//        {
//          cout  <<"Exception: "<< e.what() << '\n';
//        }


        ros::Duration diff = ros::Time::now() - t01;

        cout<<tf_pref<<" - Time for reach: "<<diff<<endl;


    }

}


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
        ros::Time t01=ros::Time::now();

        double xx,yy, tt;

        if(!_debug)
        {
            tf::StampedTransform transform;
            try{
                pos_listener.lookupTransform("/map", tf_pref+"/base_link", ros::Time(0), transform);
            }
            catch (tf::TransformException ex){
              ROS_INFO("%s",ex.what());
              //cout<<"No transform!!!\r";


              return ;
            }


            xx=transform.getOrigin().x();
            yy=transform.getOrigin().y();
            tt=tf::getYaw(transform.getRotation())/2/PI*360;

        }
        else
        {
            xx=rxr;
            yy=ryr;
            tt=rtr;
        }

        if(tt<0)
            tt+=360;




        int pos_x=((int) round((xx-or_x)/res))+robot_or.pu;

        int pos_y=((int) round((yy-or_y)/res))+robot_or.pl;

        if(map_erosionOp.at<uchar>(pos_x,pos_y)==0 && (! treated || changed) )
        {
            pos_rcv=true;
            changed=false;

            prev_x=pos_x;
            prev_y=pos_y;

            treated=true;
            gt_c=false;

            vector<cv::Mat> channels(3);

            channels[0]=map_erosionOp.clone();
            channels[1]=map_erosionOp.clone();
            channels[2]=map_erosionOp.clone();

            channels[2].at<uchar>(max(pos_x-1,0),max(pos_y-1,0))=255;
            channels[2].at<uchar>(max(pos_x-1,0),pos_y)=255;
            channels[2].at<uchar>(max(pos_x-1,0),min(pos_y+1,map_erosionOp.cols-1))=255;
            channels[2].at<uchar>(pos_x,max(pos_y-1,0))=255;
            channels[2].at<uchar>(pos_x,pos_y)=255;
            channels[2].at<uchar>(pos_x,min(pos_y+1,map_erosionOp.cols-1))=255;
            channels[2].at<uchar>(min(pos_x+1,map_erosionOp.rows-1),max(pos_y-1,0))=255;
            channels[2].at<uchar>(min(pos_x+1,map_erosionOp.rows-1),pos_y)=255;
            channels[2].at<uchar>(min(pos_x+1,map_erosionOp.rows-1),min(pos_y+1,map_erosionOp.cols-1))=255;

            channels[1].at<uchar>(max(pos_x-1,0),max(pos_y-1,0))=0;
            channels[1].at<uchar>(max(pos_x-1,0),pos_y)=0;
            channels[1].at<uchar>(max(pos_x-1,0),min(pos_y+1,map_erosionOp.cols-1))=0;
            channels[1].at<uchar>(pos_x,max(pos_y-1,0))=0;
            channels[1].at<uchar>(pos_x,pos_y)=0;
            channels[1].at<uchar>(pos_x,min(pos_y+1,map_erosionOp.cols-1))=0;
            channels[1].at<uchar>(min(pos_x+1,map_erosionOp.rows-1),max(pos_y-1,0))=0;
            channels[1].at<uchar>(min(pos_x+1,map_erosionOp.rows-1),pos_y)=0;
            channels[1].at<uchar>(min(pos_x+1,map_erosionOp.rows-1),min(pos_y+1,map_erosionOp.cols-1))=0;

            channels[0].at<uchar>(max(pos_x-1,0),max(pos_y-1,0))=0;
            channels[0].at<uchar>(max(pos_x-1,0),pos_y)=0;
            channels[0].at<uchar>(max(pos_x-1,0),min(pos_y+1,map_erosionOp.cols-1))=0;
            channels[0].at<uchar>(pos_x,max(pos_y-1,0))=0;
            channels[0].at<uchar>(pos_x,pos_y)=0;
            channels[0].at<uchar>(pos_x,min(pos_y+1,map_erosionOp.cols-1))=0;
            channels[0].at<uchar>(min(pos_x+1,map_erosionOp.rows-1),max(pos_y-1,0))=0;
            channels[0].at<uchar>(min(pos_x+1,map_erosionOp.rows-1),pos_y)=0;
            channels[0].at<uchar>(min(pos_x+1,map_erosionOp.rows-1),min(pos_y+1,map_erosionOp.cols-1))=0;


            cv::merge(channels, map_erosionOpPrintColor);


            map_label=cv::Mat::zeros(map_erosionOp.rows, map_erosionOp.cols, CV_8UC1);
            map_act=cv::Mat::zeros(map_or.rows, map_or.cols, CV_8UC1);
            map_vis=cv::Mat::zeros(map_or.rows, map_or.cols, CV_8UC1);

            ros::Duration diff = ros::Time::now() - t01;

            cout<<tf_pref<<" - Time for label: "<<diff<<endl;


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

                if(prev_x>=0 &&  prev_y>=0)
                {

                    if( prev_x==labels[i][j].x && prev_y==labels[i][j].y){

                        prev_label=i;

                    }
                }

                if(label_pos>-1 && ( (prev_label>-1) || (prev_x<0) || (prev_y<0) ) )
                    break;
            }

            if(label_pos>-1 && ( (prev_label>-1) || (prev_x<0) || (prev_y<0) ) )
                break;
        }

        if( (prev_label!=label_pos) || (prev_x<0) || (prev_y<0) || ! treated || changed)
        {

            changed=false;

            cv::Mat l_map=map_erosionOp.clone(), unreach_map, regions, act_map=map_or.clone(), vis_map=map_or.clone();


            for (int i=0;i<labels.size();i++){
                for(int j=0;j<labels[i].size();j++){
                    if( i!=label_pos ){

                        l_map.at<uchar>(labels[i][j].x,labels[i][j].y)=0;

                    }
                }
             }



            vector<cv::Mat> channels(3);

            channels[0]=map_erosionOp.clone();
            channels[1]=map_erosionOp.clone();
            channels[2]=map_erosionOp.clone();

            channels[1].at<uchar>(max(pos_x-1,0),max(pos_y-1,0))=255;
            channels[1].at<uchar>(max(pos_x-1,0),pos_y)=255;
            channels[1].at<uchar>(max(pos_x-1,0),min(pos_y+1,map_erosionOp.cols-1))=255;
            channels[1].at<uchar>(pos_x,max(pos_y-1,0))=255;
            channels[1].at<uchar>(pos_x,pos_y)=255;
            channels[1].at<uchar>(pos_x,min(pos_y+1,map_erosionOp.cols-1))=255;
            channels[1].at<uchar>(min(pos_x+1,map_erosionOp.rows-1),max(pos_y-1,0))=255;
            channels[1].at<uchar>(min(pos_x+1,map_erosionOp.rows-1),pos_y)=255;
            channels[1].at<uchar>(min(pos_x+1,map_erosionOp.rows-1),min(pos_y+1,map_erosionOp.cols-1))=255;

            channels[2].at<uchar>(max(pos_x-1,0),max(pos_y-1,0))=0;
            channels[2].at<uchar>(max(pos_x-1,0),pos_y)=0;
            channels[2].at<uchar>(max(pos_x-1,0),min(pos_y+1,map_erosionOp.cols-1))=0;
            channels[2].at<uchar>(pos_x,max(pos_y-1,0))=0;
            channels[2].at<uchar>(pos_x,pos_y)=0;
            channels[2].at<uchar>(pos_x,min(pos_y+1,map_erosionOp.cols-1))=0;
            channels[2].at<uchar>(min(pos_x+1,map_erosionOp.rows-1),max(pos_y-1,0))=0;
            channels[2].at<uchar>(min(pos_x+1,map_erosionOp.rows-1),pos_y)=0;
            channels[2].at<uchar>(min(pos_x+1,map_erosionOp.rows-1),min(pos_y+1,map_erosionOp.cols-1))=0;

            channels[0].at<uchar>(max(pos_x-1,0),max(pos_y-1,0))=0;
            channels[0].at<uchar>(max(pos_x-1,0),pos_y)=0;
            channels[0].at<uchar>(max(pos_x-1,0),min(pos_y+1,map_erosionOp.cols-1))=0;
            channels[0].at<uchar>(pos_x,max(pos_y-1,0))=0;
            channels[0].at<uchar>(pos_x,pos_y)=0;
            channels[0].at<uchar>(pos_x,min(pos_y+1,map_erosionOp.cols-1))=0;
            channels[0].at<uchar>(min(pos_x+1,map_erosionOp.rows-1),max(pos_y-1,0))=0;
            channels[0].at<uchar>(min(pos_x+1,map_erosionOp.rows-1),pos_y)=0;
            channels[0].at<uchar>(min(pos_x+1,map_erosionOp.rows-1),min(pos_y+1,map_erosionOp.cols-1))=0;


            cv::merge(channels, map_erosionOpPrintColor);





            ros::Duration diff = ros::Time::now() - t01;

            cout<<tf_pref<<" - Time for label: "<<diff<<endl;



            map_label=l_map;
            map_act=act_map;
            map_vis=vis_map;

            if(gt)
            {

                ros::Time t3=ros::Time::now();

                map_truth=brute_force_opt(map_or, map_label, defl*30);

                diff = ros::Time::now() - t3;

                cout<<tf_pref<<" - Time for Optimized brute force: "<<diff<<endl;

                gt_c=true;
            }

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
        if(gt && gt_c)
        {
            n_msg=Mat2RosMsg( map_truth , msg_rcv_pub);
            pub8.publish(n_msg);
        }
    }


    std::stringstream ss;
    ss << tf_pref+"-> Message sent: " << count;


}


std::vector<std::vector<cv::Point> >  Reach_transf::label(const cv::Mat binary, int conn)
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

std::vector<cv::Point> Reach_transf::label_seed(const cv::Mat binary, int conn, cv::Point seed)
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


void print_thread_id (int id) {
  // critical section (exclusive access to std::cout signaled by locking mtx):
  mtx.lock();
  cout << "thread #" << id << '\n';
  mtx.unlock();
}


int main(int argc, char **argv)
{
//    thread threads[10];
//    // spawn 10 threads:
//    for (int i=0; i<10; ++i)
//      threads[i] = thread(print_thread_id,i+1);

//    for (int i=0; i<10; ++i)
//        threads[i].join();



    ros::init(argc, argv, "reach");
    //debug=true;
    if(argc==2)
    {
        filename=string(argv[1]);

    }
    //else if (argc==3)
    //{
    //    filename=string(argv[1]);
    //    if(string(argv[2])==string("0"))
    //        debug=false;
    //}
    else
        return -1;


    cv::Mat robot=cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);


    if(robot.empty())
        return -2;


    for(int i=0;i<robot.rows;i++)
    {
        for(int j=0;j<robot.cols;j++)
        {
            if (robot.at<uchar>(i,j)>128)
                robot.at<uchar>(i,j)=255;
            else
                robot.at<uchar>(i,j)=0;
        }
    }



  ros::NodeHandle nh("~");


  //Reach_transf reach(nh,robot, debug);
  Reach_transf reach(nh,robot);


  ros::Rate loop_rate(10);


  while (ros::ok())
  {
    ros::spinOnce();

    reach.transf();

    reach.transf_pos();

    reach.show();

    reach.publish();

    loop_rate.sleep();
  }


  return 0;
}
