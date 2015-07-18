#include "visNC_transf.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "clustering.hpp"

#include "labelling.hpp"
#include "ray.hpp"
#include "color.hpp"
#include "vector_utils.hpp"

using namespace std;

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
static const std::string S_WINDOW = "Structuring Element";
static const std::string P_WINDOW = "ProjectedLabeled";
static const std::string R_WINDOW = "ProjectedActuation";


VisNC_transf::VisNC_transf(ros::NodeHandle nh, cv::Mat rob): Vis_transf(nh), robot(rob)
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

    this->nh_.param("theta", this->rtr, 0.0);

    this->nh_.param("angle_res", this->angle_res, 32);

    this->nh_.param("debug_angle", this->angle_debug, 0.0);

    if(this->_debug){
        cv::namedWindow(M_WINDOW);
        cv::namedWindow(E_WINDOW);
        cv::namedWindow(PE_WINDOW);
        cv::namedWindow(C_WINDOW);
        cv::namedWindow(PC_WINDOW);
        cv::namedWindow(S_WINDOW);
        cv::namedWindow(D_WINDOW);
        if(this->pos_rcv)
        {
            cv::namedWindow(L_WINDOW);
            cv::namedWindow(A_WINDOW);
            cv::namedWindow(P_WINDOW);
            cv::namedWindow(R_WINDOW);
            cv::namedWindow(V_WINDOW);
            if(this->gt && this->gt_c)
                cv::namedWindow(G_WINDOW);
        }
    }


}

VisNC_transf::~VisNC_transf()
{
    if(this->_debug){
       cv::destroyWindow(M_WINDOW);
       cv::destroyWindow(E_WINDOW);
       cv::destroyWindow(PE_WINDOW);
       cv::destroyWindow(C_WINDOW);
       cv::destroyWindow(PC_WINDOW);
       cv::destroyWindow(S_WINDOW);
       cv::destroyWindow(D_WINDOW);
       cv::destroyWindow(L_WINDOW);
       cv::destroyWindow(A_WINDOW);
       cv::destroyWindow(P_WINDOW);
       cv::destroyWindow(R_WINDOW);
       cv::destroyWindow(V_WINDOW);
       cv::destroyWindow(G_WINDOW);

    }
}


void VisNC_transf::show(void)
{
    if(this->count>0 && this->_debug){
        cv::imshow(M_WINDOW,this->map_or);
        cv::imshow(E_WINDOW,this->map_erosionOpPrintColor);
        cv::imshow(PE_WINDOW,this->map_projEros);
        cv::imshow(C_WINDOW,this->map_closeOp);
        cv::imshow(PC_WINDOW,this->map_projClose);
        cv::imshow(S_WINDOW,this->struct_elem);
        cv::imshow(D_WINDOW,this->map_debug);
        if(this->pos_rcv)
        {
            cv::imshow(L_WINDOW,this->map_label);
            cv::imshow(A_WINDOW,this->map_act);
            cv::imshow(P_WINDOW,this->map_projLabel);
            cv::imshow(R_WINDOW,this->map_projAct);
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
       cv::destroyWindow(PE_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(C_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(PC_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(S_WINDOW);
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
    }
}

void VisNC_transf::conf_space(void)
{
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

    int m_a=angleD2I(rtrd, angle_res);

    er_map=mer_map[m_a].clone();
    cl_map=mcl_map[m_a].clone();


    this->map_debug=color_print3(er_map, cl_map, or_mapN, c123, c12, c13, c23, c1, c2, c3, c0 );

    this->map_or=or_map;
    this->map_erosionOp=er_map;

    rec=cv::Rect(robot_or.pl,robot_or.pu, or_map.cols, or_map.rows);
    this->map_closeOp=cl_map(rec);

    struct_elem=robot_or.elems[m_a]*255;

    this->multi_er_map=mer_map;

    this->map_projEros=cv::Mat::zeros(map_erosionOp.rows, map_erosionOp.cols, CV_8UC1);

    for(int i=0; i<(int)mer_map.size(); i++){
        for(int j=0; j<mer_map[i].rows; j++){
            for(int k=0; k<mer_map[i].cols; k++){
                if( mer_map[i].at<uchar>(j,k)==255 )
                    this->map_projEros.at<uchar>(j,k)=255;
            }
        }
    }

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
        map_projAct=cv::Mat::zeros(map_erosionOp.rows, map_erosionOp.cols, CV_8UC1);
        map_projLabel=cv::Mat::zeros(map_erosionOp.rows, map_erosionOp.cols, CV_8UC1);
    }
    return value;
}

void VisNC_transf::visibility(cv::Point3i pos, bool proc, ros::Time t01)
{
    int pos_x=pos.x, pos_y=pos.y;

    multi_labl_map.resize(multi_er_map.size());
    for(int i=0; i<(int)multi_er_map.size(); i++){
        multi_labl_map[i]=multi_er_map[i].clone();
    }

    multi_labl_map=cluster_points(multi_labl_map, pos);

    std::vector<std::vector<cv::Point> > labels=label(this->multi_er_map[pos.z].clone()/255,8);

    unsigned int label_pos=0, prev_label=0;
    bool found_pos=false, found_prev=false;


    int prev_x=this->prev.x;
    int prev_y=this->prev.y;

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
        cv::Mat unreach_map, regions, vis_map=this->map_or.clone(), temp;

        cv::Mat l_map=cv::Mat::zeros(map_erosionOp.rows, map_erosionOp.cols, CV_8UC1);
        cv::Mat act_map=cv::Mat::zeros(map_erosionOp.rows, map_erosionOp.cols, CV_8UC1);

        multi_act_map=multiDilation(multi_labl_map, robot_or);

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

    this->prev.x=pos_x;
    this->prev.y=pos_y;
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
    angle_res=config.angle_res;
    this->_debug=config.debug;
    this->gt=config.ground_truth;
    angle_debug=config.debug_angle;
    this->rxr=config.x;
    this->ryr=config.y;
    this->rtr=config.theta;
}
