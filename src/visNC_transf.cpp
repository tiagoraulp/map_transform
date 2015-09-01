#include "visNC_transf.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "clustering.hpp"

#include "labelling.hpp"
#include "ray.hpp"
#include "color.hpp"
#include "vector_utils.hpp"
#include "unreachable.hpp"
#include "CritPoints.hpp"

using namespace std;

static const double PI = 3.141592653589793;

static const std::string M_WINDOW = "Map";
static const std::string E_WINDOW = "Erosion";
static const std::string PE_WINDOW = "ProjectedErosion";
static const std::string C_WINDOW = "Close";
static const std::string PC_WINDOW = "ProjectedClose";
static const std::string L_WINDOW = "Labelled";
static const std::string A_WINDOW = "Actuation";
static const std::string V_WINDOW = "Visibility";
static const std::string D_WINDOW = "Debug";
static const std::string G_WINDOW = "Ground_Truth";
static const std::string SR_WINDOW = "Structuring Robot";
static const std::string S_WINDOW = "Structuring Act";
static const std::string SS_WINDOW = "Structuring Sensor";
static const std::string EV_WINDOW = "Structuring Ext Vis";
static const std::string P_WINDOW = "ProjectedLabeled";
static const std::string R_WINDOW = "ProjectedActuation";
static const std::string DP_WINDOW = "Debug Pos";


VisNC_transf::VisNC_transf(ros::NodeHandle nh, cv::Mat rob, cv::Mat sens): Vis_transf(nh), robot(rob), sensor(sens)
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

    this->nh_.param("sx", scx, 50.0);
    scx/=100;
    this->nh_.param("sy", scy, 50.0);
    scy/=100;
    this->nh_.param("st", sct, 0.0);

    this->nh_.param("dx", dx, 50.0);
    dx/=100;
    this->nh_.param("dy", dy, 50.0);
    dy/=100;

    this->nh_.param("theta", this->rtr, 0.0);

    this->nh_.param("angle_res", this->angle_res, 32);

    this->nh_.param("sens_res", this->sens_res, 128);

    this->nh_.param("debug_angle", this->angle_debug, 0.0);

    sens_area.assign(sens_res, 0);
}

VisNC_transf::~VisNC_transf()
{
    if(this->_debug){
       cv::destroyWindow(M_WINDOW);
       cv::destroyWindow(E_WINDOW);
       cv::destroyWindow(PE_WINDOW);
       cv::destroyWindow(C_WINDOW);
       cv::destroyWindow(PC_WINDOW);
       cv::destroyWindow(SR_WINDOW);
       cv::destroyWindow(S_WINDOW);
       cv::destroyWindow(SS_WINDOW);
       cv::destroyWindow(EV_WINDOW);
       cv::destroyWindow(D_WINDOW);
       cv::destroyWindow(L_WINDOW);
       cv::destroyWindow(A_WINDOW);
       cv::destroyWindow(P_WINDOW);
       cv::destroyWindow(R_WINDOW);
       cv::destroyWindow(V_WINDOW);
       cv::destroyWindow(G_WINDOW);
       cv::destroyWindow(DP_WINDOW);
    }
}


void VisNC_transf::show(void)
{
    if(this->count>0 && this->_debug){
        cv::imshow(M_WINDOW,this->map_or);
        cv::imshow(PE_WINDOW,this->map_projEros);
        cv::imshow(C_WINDOW,this->map_closeOp);
        cv::imshow(PC_WINDOW,this->map_projClose);
        cv::imshow(SR_WINDOW,this->struct_elemR);
        cv::imshow(S_WINDOW,this->struct_elemA);
        cv::imshow(SS_WINDOW,this->struct_elemS);
        cv::imshow(EV_WINDOW,this->struct_elemEV);
        cv::imshow(D_WINDOW,this->map_debug);
        if(this->pos_rcv)
        {
            cv::imshow(E_WINDOW,this->map_erosionOpPrintColor);
            cv::imshow(L_WINDOW,this->map_label);
            cv::imshow(A_WINDOW,this->map_act);
            cv::imshow(P_WINDOW,this->map_projLabel);
            cv::imshow(R_WINDOW,this->map_projAct);
            cv::imshow(V_WINDOW,this->map_vis);
            cv::imshow(DP_WINDOW,this->map_debug_pos);
            if(this->gt)
            {
                cv::imshow(G_WINDOW,this->map_truth);
            }
            else
            {
                cv::destroyWindow(G_WINDOW);
                cv::waitKey(2);
            }
        }
        else
        {
            cv::imshow(E_WINDOW,map_erosionOp);

            cv::destroyWindow(DP_WINDOW);
            cv::waitKey(2);
            cv::destroyWindow(L_WINDOW);
            cv::waitKey(2);
            cv::destroyWindow(A_WINDOW);
            cv::waitKey(2);
            cv::destroyWindow(V_WINDOW);
            cv::waitKey(2);
            cv::destroyWindow(P_WINDOW);
            cv::waitKey(2);
            cv::destroyWindow(R_WINDOW);
            cv::waitKey(2);
            cv::destroyWindow(G_WINDOW);
            cv::waitKey(2);
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
       cv::destroyWindow(PE_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(C_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(PC_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(SR_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(S_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(SS_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(EV_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(D_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(L_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(A_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(P_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(R_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(V_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(G_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(DP_WINDOW);
       cv::waitKey(2);
    }
}

bool VisNC_transf::testConf(void)
{
    if(robot_or.pt2.size()>0)
    {
        cv::Point pt=robot_or.pt2[0];
        if(pt.x>=0 && pt.x<robot_or.elems[0].rows && pt.y>=0 && pt.y<robot_or.elems[0].cols)
        {
            if(robot_or.elems[0].at<uchar>(pt.x,pt.y)!=0)
            {
                return true;
            }
        }
    }
    return false;
}

bool VisNC_transf::conf_space(void)
{
    cv::Mat or_map, er_map, cl_map, or_mapN;

    or_map=this->cv_map.clone();
    this->msg_rcv_pub=this->msg_rcv;


    cv::Mat elem = this->robot.clone();

    cv::Point2f pt=cv::Point2f(elem.rows*rcx,elem.cols*rcy);

    cv::Point2f pt2=cv::Point2f(elem.rows*dx,elem.cols*dy);

    robot_or=multiRobot(elem, pt,this->rct, this->rinfl, this->angle_res, pt2);

    cv::Point2f pt3=cv::Point2f(sensor.rows*scx,sensor.cols*scy);
    pt2=pt-pt2+pt3;

    sensor_or=multiSensor(this->sensor.clone(), pt3,this->sct, this->sdefl, this->angle_res, pt2);

    cv::copyMakeBorder(or_map,or_mapN,this->robot_or.pu,this->robot_or.pb,this->robot_or.pl,robot_or.pr,cv::BORDER_CONSTANT,cv::Scalar(0));

    sensor_or.pu=robot_or.pu;
    sensor_or.pl=robot_or.pl;
    sensor_or.pb=robot_or.pb;
    sensor_or.pr=robot_or.pr;

    vector<cv::Mat> mer_map=multiErosion(or_mapN, robot_or);

    this->multi_er_map=mer_map;
    this->map_or=or_map;

    double rtrd=angle_debug;

    int m_a=angleD2I(rtrd, angle_res);

    er_map=mer_map[m_a].clone();

    this->map_erosionOp=er_map;

    this->map_projEros=cv::Mat::zeros(map_erosionOp.rows, map_erosionOp.cols, CV_8UC1);

    for(int i=0; i<(int)mer_map.size(); i++){
        for(int j=0; j<mer_map[i].rows; j++){
            for(int k=0; k<mer_map[i].cols; k++){
                if( mer_map[i].at<uchar>(j,k)==255 )
                    this->map_projEros.at<uchar>(j,k)=255;
            }
        }
    }

    unsigned char color[3]={255,0,0};
    unsigned char color2[3]={0,255,0};

    struct_elemR=printPoint(robot_or.elems[m_a],robot_or.pt, color);
    struct_elemS=printPoint(sensor_or.elems[m_a],sensor_or.pt2[m_a], color2);

    multiMerge(robot_or, sensor_or, robot_act, sensor_ev);

    struct_elemA=printPoint(robot_act.elems[m_a],robot_act.pt, color);
    struct_elemA=printPoint(struct_elemA, robot_or.pt2[m_a], color2);


    if(!testConf())
    {
        struct_elemEV=cv::Mat::zeros(struct_elemS.rows,struct_elemS.cols,CV_8UC1);
        map_debug=cv::Mat::zeros(map_erosionOp.rows,map_erosionOp.cols,CV_8UC1);
        map_closeOp=cv::Mat::zeros(map_or.rows,map_or.cols,CV_8UC1);
        map_projClose=cv::Mat::zeros(map_or.rows,map_or.cols,CV_8UC1);
        return false;
    }

    calcSensArea(sensor_ev);

    struct_elemEV=printPoint(sensor_ev.elems[m_a],sensor_ev.pt, color);
    struct_elemEV=printPoint(struct_elemEV,sensor_or.pt2[m_a], color2);

    vector<cv::Mat> mcl_map=multiDilation(mer_map, robot_act);

    cl_map=mcl_map[m_a].clone();

    unsigned char c123[3]={0,0,255};
    unsigned char c12[3]={255,255,0};
    unsigned char c13[3]={255,0,255};
    unsigned char c23[3]={0,255,255};
    unsigned char c1[3]={0,255,0};
    unsigned char c2[3]={255,255,255};
    unsigned char c3[3]={255,0,0};
    unsigned char c0[3]={0,0,0};


    this->map_debug=color_print3(er_map, cl_map, or_mapN, c123, c12, c13, c23, c1, c2, c3, c0 );


    rec=cv::Rect(robot_or.pl,robot_or.pu, or_map.cols, or_map.rows);
    this->map_closeOp=cl_map(rec);

    this->map_projClose=cv::Mat::zeros(map_erosionOp.rows, map_erosionOp.cols, CV_8UC1);

    for(int i=0; i<(int)mcl_map.size(); i++){
        for(int j=0; j<mcl_map[i].rows; j++){
            for(int k=0; k<mcl_map[i].cols; k++){
                if( mcl_map[i].at<uchar>(j,k)==255 )
                    this->map_projClose.at<uchar>(j,k)=255;
            }
        }
    }
    this->map_projClose=map_projClose(rec);

    return true;
}

void VisNC_transf::calcSensArea(Elem sens)
{
    sens_area.assign(sens_res, 0);

    for(int i=0; i<sens.elems[0].rows; i++)
    {
        for(int j=0; j<sens.elems[0].cols; j++)
        {
            if(sens.elems[0].at<uchar>(i,j)!=0)
            {
                int x=i-sens.pt.x, y=j-sens.pt.y;

                if(x==0 && y==0)
                {
                    sens_area[0]+=1;
                }
                else
                {
                    int angle=angleD2I(atan2(y,x)/PI*180, sens_res);

                    sens_area[angle]+=1;
                }
            }
        }
    }
}

void VisNC_transf::getPosition(cv::Point3d &p)
{
    p.x=this->rxr/100.0*this->map_or.rows*this->res;
    p.y=this->ryr/100.0*this->map_or.cols*this->res;
    p.z=this->rtr;
}

void VisNC_transf::get2DPosition(cv::Point3i& pos, cv::Point3d p)
{
    pos.x=(int) round((p.x-this->or_x)/this->res)+this->robot_or.pu;
    pos.y=(int) round((p.y-this->or_y)/this->res)+this->robot_or.pl;

    pos.x=boundPos(pos.x, this->map_or.rows+this->robot_or.pb+this->robot_or.pu);
    pos.y=boundPos(pos.y,this-> map_or.cols+this->robot_or.pl+this->robot_or.pr);

    pos.z=angleD2I(p.z, angle_res);
}

bool VisNC_transf::valid_pos(cv::Point3i pos)
{
    bool value=(multi_er_map[pos.z].at<uchar>(pos.x,pos.y)!=0);
    if(!value)
    {
        map_projAct=cv::Mat::zeros(map_or.rows, map_or.cols, CV_8UC1);
        map_projLabel=cv::Mat::zeros(map_erosionOp.rows, map_erosionOp.cols, CV_8UC1);
        map_debug_pos=cv::Mat::zeros(map_closeOp.rows, map_closeOp.cols, CV_8UC1);
    }
    return value;
}

void VisNC_transf::visibility(cv::Point3i pos, bool proc, ros::Time t01)
{
    multi_labl_map.resize(multi_er_map.size());
    for(int i=0; i<(int)multi_er_map.size(); i++){
        multi_labl_map[i]=multi_er_map[i].clone();
    }

    multi_labl_map=cluster_points(multi_labl_map, pos);

    bool labels=true;

    if( prev.z<(int)multi_labl_map.size() && prev.z>=0 )
    {
        if( prev.x<multi_labl_map[prev.z].rows && prev.x>=0 && prev.y<multi_labl_map[prev.z].cols && prev.y>=0  )
        {
            if( multi_labl_map[prev.z].at<uchar>(prev.x, prev.y)!=0 )
            {
                labels=false;
            }
        }
    }

    if( labels || (prev.x<0) || (prev.y<0) || proc)
    {
        cv::Mat regions, vis_map=this->map_or.clone(), temp;

        cv::Mat l_map=cv::Mat::zeros(map_erosionOp.rows, map_erosionOp.cols, CV_8UC1);
        cv::Mat act_map=cv::Mat::zeros(map_erosionOp.rows, map_erosionOp.cols, CV_8UC1);

        multi_act_map=multiDilation(multi_labl_map, robot_act);

        for(int i=0; i<(int)multi_labl_map.size(); i++){
            for(int j=0; j<multi_labl_map[i].rows; j++){
                for(int k=0; k<multi_labl_map[i].cols; k++){
                    if( multi_labl_map[i].at<uchar>(j,k)==255 )
                        l_map.at<uchar>(j,k)=255;
                    if( multi_act_map[i].at<uchar>(j,k)==255 )
                        act_map.at<uchar>(j,k)=255;
                }
            }
        }

        int m_a=angleD2I(angle_debug, angle_res);

        temp=multi_act_map[m_a].clone();
        this->map_label=multi_labl_map[m_a].clone();
        this->map_act=temp(rec);
        this->map_projLabel=l_map;
        this->map_projAct=act_map(rec);
        this->map_vis=vis_map;

        Unreachable unreach(map_or, act_map(rec));

        map_debug_pos=unreach.unreach_map;

        unreach.getFrontiers2();

        regions=unreach.regions;

        CritPointsAS critP(map_or, multi_labl_map, sensor_ev, sens_area);

        for (unsigned int k=0;k<unreach.clusters.size();k++){//2;k++){//
            for(unsigned int ff=0;ff<unreach.clusters[k].size();ff++)
            {
                vector<cv::Point> frontier=unreach.clusters[k][ff].cluster;

                if(frontier.size()>0)
                {
                    cv::Point3i crit=critP.find_crit_point(unreach.clusters[k][ff]);

                    cout<<unreach.clusters[k][ff].extremes.size()<<endl;

                    map_debug_pos.at<uchar>(crit.x, crit.y)=0;

                }
            }
        }

        unsigned char color[3]={0,255,0};

        map_erosionOpPrintColor=printPoint(map_erosionOp, cv::Point(pos.x,pos.y), color);

        ros::Duration diff = ros::Time::now() - t01;

        ROS_INFO("%s - Time for visibility: %f", this->tf_pref.c_str(), diff.toSec());

        if(this->gt)
        {
            ros::Time t3;

            vector<cv::Point3i> reach_list;
            reach_list.clear();

            for(unsigned int a=0; a<multi_labl_map.size();a++)
            {
                for(int i=0; i<multi_labl_map[a].rows;i++)
                {
                    for(int j=0; j<multi_labl_map[a].cols;j++)
                    {
                        if(multi_labl_map[a].at<uchar>(i,j)!=0)
                            reach_list.push_back(cv::Point3i(i,j,a));
                    }
                }
            }

            /////////////////

//            t3=ros::Time::now();

//            this->map_truth=brute_force(map_or, multi_labl_map , sensor_or, true, map_projAct);

//            diff = ros::Time::now() - t3;

//            ROS_INFO("%s - Time for Optimized brute force sq with act: %f", this->tf_pref.c_str(), diff.toSec());

//            cv::imshow("TestSQwA",this->map_truth);
//            cv::waitKey(3);

//            ////////////////////////

//            t3=ros::Time::now();

//            this->map_truth=brute_force(map_or, multi_labl_map , sensor_or, true);

//            diff = ros::Time::now() - t3;

//            ROS_INFO("%s - Time for Optimized brute force sq without act: %f", this->tf_pref.c_str(), diff.toSec());

//            cv::imshow("TestSQ",this->map_truth);
//            cv::waitKey(3);

//            ////////////////////////


//            t3=ros::Time::now();

//            this->map_truth=brute_force(map_or, multi_labl_map , sensor_or, false, map_projAct);

//            diff = ros::Time::now() - t3;

//            ROS_INFO("%s - Time for Optimized brute force grid with act: %f", this->tf_pref.c_str(), diff.toSec());

//            cv::imshow("TestGwA",this->map_truth);
//            cv::waitKey(3);

//            ////////////////////////

//            t3=ros::Time::now();

//            this->map_truth=brute_force(map_or, multi_labl_map , sensor_or, false);

//            diff = ros::Time::now() - t3;

//            ROS_INFO("%s - Time for Optimized brute force grid without act: %f", this->tf_pref.c_str(), diff.toSec());

//            cv::imshow("TestG",this->map_truth);
//            cv::waitKey(3);

//            ////////////////////////

            t3=ros::Time::now();

            this->map_truth=brute_force(map_or, reach_list , sensor_or, false, map_projAct);

            diff = ros::Time::now() - t3;

            ROS_INFO("%s - Time for Optimized brute force list with act: %f", this->tf_pref.c_str(), diff.toSec());

            //cv::imshow("TestLwA",this->map_truth);
            //cv::waitKey(3);

            ////////////////////////

//            t3=ros::Time::now();

//            this->map_truth=brute_force(map_or, reach_list , sensor_or, false);

//            diff = ros::Time::now() - t3;

//            ROS_INFO("%s - Time for Optimized brute force list without act: %f", this->tf_pref.c_str(), diff.toSec());

//            cv::imshow("TestL",this->map_truth);
//            cv::waitKey(3);

//            ////////////////////////

            this->gt_c=true;
        }

    }

    prev=pos;
}


void VisNC_transf::update_config(map_transform::ParametersncConfig config)
{
    this->changed=true;
    this->changed2=true;
    rinfl=config.rinfl/100;
    sdefl=config.sdefl/100;
    rcx=config.rx/100;
    rcy=config.ry/100;
    rct=config.rt;
    scx=config.sx/100;
    scy=config.sy/100;
    dx=config.dx/100;
    dy=config.dy/100;
    sct=config.st;
    angle_res=config.angle_res;
    sens_res=config.sens_res;
    this->_debug=config.debug;
    this->gt=config.ground_truth;
    angle_debug=config.debug_angle;
    this->rxr=config.x;
    this->ryr=config.y;
    this->rtr=config.theta;
}


void VisNC_transf::clearImgs(void)
{
    map_debug_pos=cv::Mat::zeros(map_or.rows, map_or.cols, CV_8UC1);
    map_label=cv::Mat::zeros(map_erosionOp.rows, map_erosionOp.cols, CV_8UC1);
    map_projLabel=map_label;
    map_erosionOpPrintColor=map_label;
    map_act=map_debug_pos;
    map_projAct=map_debug_pos;
    map_vis=map_debug_pos;
    map_truth=map_debug_pos;
}
