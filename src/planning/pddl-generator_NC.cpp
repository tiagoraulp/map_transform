#include "ros/ros.h"
#include <ros/package.h>
#include <signal.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <opencv2/core/core.hpp>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/MarkerArray.h"
#include "vector_utils.hpp"
#include "Astar.hpp"
#include "ray.hpp"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "points_conversions.hpp"

static const double PI = 3.141592653589793;

#define C_MAP 0
#define E_MAP 1
#define A_MAP 2
#define R_MAP 3
#define PC_MAP 4
#define PE_MAP 5
#define PA_MAP 6
#define PR_MAP 7
#define O_MAP 8

#define MC_MAP 0
#define ME_MAP 1
#define MA_MAP 2
#define MR_MAP 3
#define M_MAP 4

#define ROB_STR 0
#define ACT_STR 1
#define STRUCTS 2

class PddlGenNC{
private:
    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> subs;
    std::vector<ros::Subscriber> subs_act;
    std::vector<ros::Subscriber> subs_multi;
    std::vector<ros::Subscriber> subs_multi_str;
    std::vector<ros::Subscriber> subs_rob_center;
    std::vector<ros::Publisher> pub_mar;
    std::vector<ros::Publisher> pubs;
    std::vector<nav_msgs::Path> paths;
    bool output;
    std::vector<std::vector<std::vector<bool> > >  msg_rcv;
    std::vector<cv::Mat_<int> >  msg_rcv_act;
    std::vector<std::vector<std::vector<std::vector<bool> > > > msg_rcv_multi;
    std::vector<std::vector<std::vector<std::vector<bool> > > > msg_rcv_multi_str;
    std::vector<int> countM;
    std::vector<bool> map_rcv;
    std::vector<bool> map_rcv_act;
    std::vector<bool> map_rcv_multi;
    std::vector<bool> map_rcv_multi_str;
    std::vector<cv::Point2i> rob_center;
    float res;
    int width,height;
    cv::Mat or_map;
    int jump, nrobots;
    std::vector<int> rs;
    tf::TransformListener pos_listener;
    std::vector<cv::Point2i> wps;
    std::vector<cv::Point3i> wps3D;
    std::vector<std::vector<std::vector<bool> > > graph;
    std::vector<std::vector<std::vector<bool> > > actuable;
    std::vector<std::vector<long int> > wp_n2i;
    std::vector<std::vector<std::vector<long int> > > wp_n2i3D;
    std::string index2string(int index);
    void rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg, int index);
    void rcv_map_act(const std_msgs::Int16MultiArray::ConstPtr& msg, int index);
    void rcv_map_multi(const std_msgs::UInt8MultiArray::ConstPtr& msg, int index);
    void rcv_map_multi_str(const std_msgs::UInt8MultiArray::ConstPtr& msg, int index);
    void rcv_rob_center(const std_msgs::Int16MultiArray::ConstPtr& msg, int index);
    bool getMapValue(int n, int i, int j);
    int getActMapValue(int n, int i, int j);
    bool getMultiMapValue(int n, int l, int i, int j);
    bool getStructValue(int n, int l, int i, int j);
    bool getStructCenteredValue(int n, int l, int i, int j);
    cv::Point2i getStructLower(int n);
    cv::Point2i getStructUpper(int n);
    bool valid(int i, int j, int l,int r);
    bool connection(int i, int j, int l, int in, int jn, int ln, int r);
    bool connection_ray(int i, int j, int l, int in, int jn, int r_e, int r_v, bool strict=true);
    bool procPaths(std::vector<std::string> file);
    bool allMapsReceived(void);
    bool allActMapsReceived(void);
    bool allMultiMapsReceived(void);
    bool allMultiStrMapsReceived(void);
    bool allRobCenterReceived(void);
    bool validWaypoint(unsigned int i, unsigned int j);
    bool feasibleWaypoint(unsigned int i, unsigned int j);
public:
    PddlGenNC(ros::NodeHandle nh): nh_(nh){
        nh_.param("nrobots", nrobots, 2);
        countM.assign(O_MAP*nrobots+1,0);
        map_rcv.assign(O_MAP*nrobots+1,false);
        msg_rcv.resize(O_MAP*nrobots+1);
        map_rcv_act.assign(nrobots,false);
        msg_rcv_act.resize(nrobots);
        map_rcv_multi.assign(M_MAP*nrobots,false);
        msg_rcv_multi.resize(M_MAP*nrobots);
        map_rcv_multi_str.assign(STRUCTS*nrobots,false);
        msg_rcv_multi_str.resize(STRUCTS*nrobots);
        rob_center.assign(nrobots,cv::Point2i(-1,-1));
        wps.clear();
        wps3D.clear();
        graph.clear();
        actuable.clear();

        subs.resize(O_MAP*nrobots+1);
        for(int i=0;i<O_MAP;i++){
            for(int j=0;j<nrobots;j++){
                std::string topic="/robot_"+std::to_string(j)+"/"+index2string(i)+"_map";
                subs[i*nrobots+j]=nh_.subscribe<nav_msgs::OccupancyGrid>(topic, 1, boost::bind(&PddlGenNC::rcv_map,this,_1,i*nrobots+j));
            }
        }
        subs[subs.size()-1] = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, boost::bind(&PddlGenNC::rcv_map,this,_1,O_MAP*nrobots));

        subs_act.resize(nrobots);
        for(int j=0;j<nrobots;j++){
            std::string topic="/robot_"+std::to_string(j)+"/"+"act_map";
            subs_act[j]=nh_.subscribe<std_msgs::Int16MultiArray>(topic, 1,  boost::bind(&PddlGenNC::rcv_map_act,this,_1,j));
        }

        subs_multi.resize(M_MAP*nrobots);
        for(int i=0;i<M_MAP;i++){
            for(int j=0;j<nrobots;j++){
                std::string topic="/robot_"+std::to_string(j)+"/"+index2string(i)+"_multi";
                subs_multi[i*nrobots+j]=nh_.subscribe<std_msgs::UInt8MultiArray>(topic, 1, boost::bind(&PddlGenNC::rcv_map_multi,this,_1,i*nrobots+j));
            }
        }

        subs_multi_str.resize(STRUCTS*nrobots);
        for(int i=0;i<STRUCTS;i++){
            for(int j=0;j<nrobots;j++){
                if(i==0){
                    std::string topic="/robot_"+std::to_string(j)+"/"+"rob_struct";
                    subs_multi_str[i*nrobots+j]=nh_.subscribe<std_msgs::UInt8MultiArray>(topic, 1, boost::bind(&PddlGenNC::rcv_map_multi_str,this,_1,i*nrobots+j));
                }
                else{
                    std::string topic="/robot_"+std::to_string(j)+"/"+"act_struct";
                    subs_multi_str[i*nrobots+j]=nh_.subscribe<std_msgs::UInt8MultiArray>(topic, 1, boost::bind(&PddlGenNC::rcv_map_multi_str,this,_1,i*nrobots+j));
                }
            }
        }

        subs_rob_center.resize(nrobots);
        for(int j=0;j<nrobots;j++){
            std::string topic="/robot_"+std::to_string(j)+"/"+"rob_center";
            subs_rob_center[j]=nh_.subscribe<std_msgs::Int16MultiArray>(topic, 1, boost::bind(&PddlGenNC::rcv_rob_center,this,_1,j));
        }

        output=false;
        pubs.resize(nrobots);
        for(int j=0;j<nrobots;j++){
            std::string topic="path"+std::to_string(j);
            pubs[j]=nh_.advertise<nav_msgs::Path>(topic, 1,true);
        }
        pub_mar.resize(nrobots*2+1);
        pub_mar[0] = nh_.advertise<visualization_msgs::MarkerArray>("waypoints", 1,true);
        for(int j=0;j<nrobots;j++){
            std::string topic="connected_"+std::to_string(j);
            pub_mar[1+2*j]=nh_.advertise<visualization_msgs::MarkerArray>(topic, 1,true);
            topic="visible_"+std::to_string(j);
            pub_mar[1+2*j+1]=nh_.advertise<visualization_msgs::MarkerArray>(topic, 1,true);
        }
        rs.resize(nrobots);
        for(int j=0;j<nrobots;j++){
            std::string param="/robot_"+std::to_string(j)+"/visibility/infl";
            nh_.param(param, rs[j], 5);
        }
        paths.resize(nrobots);
        nh_.param("jump",jump, 5);
    }
    bool run(void);
    void readOutput(void);
    void publish(void);
};

std::string PddlGenNC::index2string(int index){
    switch (index) {
        case C_MAP:
            return "c";
        case E_MAP:
            return "e";
        case A_MAP:
            return "a";
        case R_MAP:
            return "r";
        case PC_MAP:
            return "pc";
        case PE_MAP:
            return "pe";
        case PA_MAP:
            return "pa";
        case PR_MAP:
            return "pr";
        default:
            return "c";
    }
}

void PddlGenNC::rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg, int index){
    if(index==(nrobots*O_MAP))
        or_map = cv::Mat(msg->info.width, msg->info.height, CV_8UC1);

    msg_rcv[index].assign(msg->info.width, std::vector<bool>(msg->info.height,false));

    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
            if(*mapDataIterC == 0){
                msg_rcv[index][j][i]=true;
                if(index==(nrobots*4))
                    or_map.at<uchar>(j,i) = 255;
            }
            else{
                msg_rcv[index][j][i]=false;
                if(index==(nrobots*4))
                    or_map.at<uchar>(j,i) = 0;
            }
            mapDataIterC++;
        }
    }
    map_rcv[index]=true;
    countM[index]=countM[index]+1;
    res=msg->info.resolution;
    width=msg->info.width;
    height=msg->info.height;
}

void PddlGenNC::rcv_map_act(const std_msgs::Int16MultiArray::ConstPtr& msg, int index){
    msg_rcv_act[index]=cv::Mat_<int>::ones(msg->layout.dim[0].size, msg->layout.dim[1].size)*-1;

    for(unsigned int i=0;i<msg->layout.dim[0].size;i++){
        for(unsigned int j=0;j<msg->layout.dim[1].size;j++){
                msg_rcv_act[index](i,j)=msg->data[msg->layout.data_offset+msg->layout.dim[1].stride*i+j];
        }
    }
    map_rcv_act[index]=true;
}

void PddlGenNC::rcv_map_multi(const std_msgs::UInt8MultiArray::ConstPtr& msg, int index){
    msg_rcv_multi[index].assign(msg->layout.dim[0].size, std::vector<std::vector<bool> >(msg->layout.dim[1].size,std::vector<bool>(msg->layout.dim[2].size,false)));
    for(unsigned int l=0;l<msg->layout.dim[0].size;l++){
        for(unsigned int i=0;i<msg->layout.dim[1].size;i++){
            for(unsigned int j=0;j<msg->layout.dim[2].size;j++){
                unsigned char value=msg->data[msg->layout.data_offset+msg->layout.dim[1].stride*l+msg->layout.dim[2].stride*i+j];
                if(value!=0){
                    msg_rcv_multi[index][l][i][j]=true;
                }
                else{
                    msg_rcv_multi[index][l][i][j]=false;
                }
            }
        }
    }
    map_rcv_multi[index]=true;
}

void PddlGenNC::rcv_map_multi_str(const std_msgs::UInt8MultiArray::ConstPtr& msg, int index){
    msg_rcv_multi_str[index].assign(msg->layout.dim[0].size, std::vector<std::vector<bool> >(msg->layout.dim[1].size,std::vector<bool>(msg->layout.dim[2].size,false)));
    for(unsigned int l=0;l<msg->layout.dim[0].size;l++){
        for(unsigned int i=0;i<msg->layout.dim[1].size;i++){
            for(unsigned int j=0;j<msg->layout.dim[2].size;j++){
                unsigned char value=msg->data[msg->layout.data_offset+msg->layout.dim[1].stride*l+msg->layout.dim[2].stride*i+j];
                if(value!=0){
                    msg_rcv_multi_str[index][l][i][j]=true;
                }
                else{
                    msg_rcv_multi_str[index][l][i][j]=false;
                }
            }
        }
    }
    map_rcv_multi_str[index]=true;
}

void PddlGenNC::rcv_rob_center(const std_msgs::Int16MultiArray::ConstPtr& msg, int index){
    if(((msg->layout.dim.size())!=1) || ((msg->layout.dim[0].size)!=2))
        return;
    rob_center[index]=cv::Point2i(msg->data[msg->layout.data_offset+0],msg->data[msg->layout.data_offset+1]);
}

bool PddlGenNC::getMapValue(int n, int i, int j){
    if( (n>=0) && (n<(int)msg_rcv.size()) && (i>=0) && (i<(int)msg_rcv[n].size()) && (j>=0) && (j<(int)msg_rcv[n][i].size()))
        return msg_rcv[n][i][j];
    else
        return false;
}

int PddlGenNC::getActMapValue(int n, int i, int j){
    if( (n>=0) && (n<(int)msg_rcv_act.size()) && (i>=0) && (i<(int)msg_rcv_act[n].rows) && (j>=0) && (j<(int)msg_rcv_act[n].cols))
        return msg_rcv_act[n](i,j);
    else
        return -1;
}

bool PddlGenNC::getMultiMapValue(int n, int l, int i, int j){
    if( (n>=0) && (n<(int)msg_rcv_multi.size()) && (l>=0) && (l<(int)msg_rcv_multi[n].size()) && (i>=0) && (i<(int)msg_rcv_multi[n][l].size()) && (j>=0) && (j<(int)msg_rcv_multi[n][l][i].size()))
        return msg_rcv_multi[n][l][i][j];
    else
        return false;
}

bool PddlGenNC::getStructValue(int n, int l, int i, int j){
    if( (n>=0) && (n<(int)msg_rcv_multi_str.size()) && (l>=0) && (l<(int)msg_rcv_multi_str[n].size()) && (i>=0) && (i<(int)msg_rcv_multi_str[n][l].size()) && (j>=0) && (j<(int)msg_rcv_multi_str[n][l][i].size()))
        return msg_rcv_multi_str[n][l][i][j];
    else
        return false;
}

bool PddlGenNC::getStructCenteredValue(int n, int l, int i, int j){
    if(n<0 || n>=(int)rob_center.size())
        return false;
    return getStructValue(n,l,i-rob_center[n].x,j-rob_center[n].y);
}

cv::Point2i PddlGenNC::getStructLower(int n){
    if(n<0 || n>=(int)rob_center.size())
        return cv::Point2i(0,0);
    return cv::Point2i(rob_center[n].x,rob_center[n].y);
}

cv::Point2i PddlGenNC::getStructUpper(int n){
    if(n<0 || n>=(int)rob_center.size() || msg_rcv_multi_str[n].size()==0 || msg_rcv_multi_str[n][0].size()==0 || msg_rcv_multi_str[n][0][0].size()==0)
        return cv::Point2i(0,0);
    return cv::Point2i(msg_rcv_multi_str[n][0].size()-1-rob_center[n].x,msg_rcv_multi_str[n][0][0].size()-1-rob_center[n].y);
}

std::string convertG2S(cv::Point2i pt){
    std::string str;
    str.append(std::to_string(pt.x)).append("_").append(std::to_string(pt.y));
    return str;
}

std::string convertG2S(cv::Point3i pt){
    std::string str;
    str.append(std::to_string(pt.x)).append("_").append(std::to_string(pt.y)).append("_").append(std::to_string(pt.z));
    return str;
}

void write_preamble(std::stringstream & str, int nr, int nw, std::vector<cv::Point2i> names,int nw3D, std::vector<cv::Point3i> names3D){
    str<<"(define (problem robotprob1) (:domain robot-building)"<<std::endl<<std::endl;
    str<<"(:objects\n\t";
    for(int i=0;i<nr;i++){
     str<<"robot"<<i+1<<" ";
    }
    str<<"- robot"<<std::endl<<"\t";
    for(int i=0;i<nw;i++){
        str<<"waypoint"<<convertG2S(names[i])<<" ";
    }
    for(int i=0;i<nw3D;i++){
        str<<"waypoint"<<convertG2S(names3D[i])<<" ";
    }
    str<<"- waypoint"<<std::endl<<")"<<std::endl<<std::endl;
}

void write_graph(std::stringstream & str, std::vector<std::vector<std::vector<bool> > > graph, std::vector<cv::Point3i> names){
    str<<"(:init\n";
    for(unsigned int i=0; i<graph.size(); i++)
        for(unsigned int j=0; j<graph[i].size(); j++)
            for(unsigned int k=0; k<graph[i][j].size(); k++)
                if(graph[i][j][k])
                    str<<"\t(connected robot"<<i+1<<" waypoint"<<convertG2S(names[j])<<" waypoint"<<convertG2S(names[k])<<")"<<std::endl;
    str<<std::endl;
}

void write_visible(std::stringstream & str, std::vector<std::vector<std::vector<bool> > > visible, std::vector<cv::Point> names, std::vector<cv::Point3i> names3D){
    for(unsigned int i=0; i<visible.size(); i++)
        for(unsigned int j=0; j<visible[i].size(); j++)
            for(unsigned int k=0; k<visible[i][j].size(); k++)
                if(visible[i][j][k])
                    str<<"\t(visible robot"<<i+1<<" waypoint"<<convertG2S(names3D[j])<<" waypoint"<<convertG2S(names[k])<<")"<<std::endl;
    str<<std::endl;
}

void write_initRobots(std::stringstream & str, int nr, std::vector<long int> initials, std::vector<cv::Point3i> names){
    for(int i=0; i<nr; i++){
        if(initials[i]>=0 && (initials[i]<((long int)names.size()))){
            str<<"\t(at robot"<<i+1<< " waypoint"<<convertG2S(names[initials[i]])<<")"<<std::endl;
        }
        else{
            str<<"\t(at robot"<<i+1<< " waypoint"<<")"<<std::endl;
        }
    }
    str<<std::endl;

    for(int i=0; i<nr; i++)
        str<<"\t(available robot"<<i+1<<")"<<std::endl;
    str<<")"<<std::endl<<std::endl;
}

void write_goals(std::stringstream & str, std::vector<long int> goals, std::vector<cv::Point2i> names){
    str<<"(:goal (and"<<std::endl;
    for(unsigned int i=0;i<goals.size();i++){
        str<<"\t(visited waypoint"<<convertG2S(names[goals[i]])<<")"<<std::endl;
    }
    str<<"\t)\n)\n)";
}

void write_goals(std::stringstream & str, int goals, std::vector<cv::Point2i> names){
    str<<"(:goal (and"<<std::endl;
    for(int i=0;i<goals;i++){
        str<<"\t(visited waypoint"<<convertG2S(names[i])<<")"<<std::endl;
    }
    str<<"\t)\n)\n)";
}

void write_goals_UT(std::stringstream & str, std::vector<long int> goalsUT, std::vector<cv::Point2i> names){
    str<<"(:notgoal (and"<<std::endl;
    for(unsigned int i=0;i<goalsUT.size();i++){
        str<<"\t(visited waypoint"<<convertG2S(names[goalsUT[i]])<<")"<<std::endl;
    }
    str<<"\t)\n)";
}

void write_goals_GA(std::stringstream & str, std::vector<std::vector<long int> > goalsR, std::vector<cv::Point2i> names){
    str<<"("<<std::endl;
    unsigned int nr=goalsR.size();
    for(unsigned int r=0; r<nr; r++){
        if(goalsR[r].size()>0){
            str<<"\t(#robot"<<r+1<<" ";
            for(unsigned int i=0;i<goalsR[r].size();i++){
                str<<"(visited waypoint"<<convertG2S(names[goalsR[r][i]])<<") ";
            }
            str<<")";
            if(r!=(nr-1))
                str<<";;";
            str<<std::endl;
        }
    }
    str<<")";
}

void write_goals_GAP(std::stringstream & str, std::vector<std::vector<long int> > goalsRP, std::vector<cv::Point2i> names){
    str<<"("<<std::endl;
    unsigned int nr=goalsRP.size();
    for(unsigned int r=0; r<nr; r++){
        if(goalsRP[r].size()>0){
            str<<"\t(#robot"<<r+1<<" ";
            for(unsigned int i=0;i<goalsRP[r].size();i++){
                str<<"(visited waypoint"<<convertG2S(names[goalsRP[r][i]])<<") ";
            }
            str<<")";
            if(r!=(nr-1))
                str<<";;";
            str<<std::endl;
        }
    }
    str<<")";
}

void write_goals_GAP_Heur(std::stringstream & str, std::vector<std::vector<long int> > goalsRPH){
    str<<"("<<std::endl;
    unsigned int nr=goalsRPH.size();
    for(unsigned int r=0; r<nr; r++){
        if(goalsRPH[r].size()>0){
            str<<"\t(#robot"<<r+1<<" ";
            for(unsigned int i=0;i<goalsRPH[r].size();i++){
                str<<"("<<goalsRPH[r][i]<<") ";
            }
            str<<")";
            if(r!=(nr-1))
                str<<";;";
            str<<std::endl;
        }
    }
    str<<")";
}

void write_goals_GA_P_Heur(std::stringstream & str, std::vector<std::vector<long int> > goalsRP, std::vector<cv::Point2i> names, std::vector<std::vector<long int> > goalsRPH){
    if( (goalsRP.size() != goalsRPH.size()) || (goalsRP.size()==0) )
        return;
    for(unsigned int r=0; r<goalsRP.size(); r++){
        if(goalsRP[r].size()!= goalsRPH[r].size())
                return;
    }
    str<<"("<<std::endl;
    unsigned int nr=goalsRP.size();
    for(unsigned int r=0; r<nr; r++){
        if(goalsRP[r].size()>0){
            str<<"\t(robot"<<r+1<<" ";
            for(unsigned int i=0;i<goalsRP[r].size();i++){
                str<<"((visited waypoint"<<convertG2S(names[goalsRP[r][i]])<<") "<<goalsRPH[r][i]<<") ";
            }
            str<<")";
            str<<std::endl;
        }
    }
    str<<")";
}

int win=4;
float adj=1.2;


bool PddlGenNC::valid(int i, int j,int l, int r){
    if(!getMultiMapValue(r,l,i,j)){
        bool stop=false;
        for(int ii=std::max(i-win,0); ii<=std::min(i+win,(int)msg_rcv_multi[r][l].size()-1); ii++){
            for(int jj=std::max(j-win,0); jj<=std::min(j+win,(int)msg_rcv_multi[r][l][ii].size()-1); jj++){
                if(getMultiMapValue(r,l,ii,jj)){
                    stop=true;
                    break;
                }
            }
            if(stop)
                break;
        }
        if(!stop)
            return false;
    }
    return true;
}

bool PddlGenNC::connection(int i, int j, int l, int in, int jn, int ln, int r){
    if(!getMultiMapValue(r,l,i,j)){
        bool stop=false;
        for(int ii=std::max(i-win,0); ii<=std::min(i+win,(int)msg_rcv_multi[r][l].size()-1); ii++){
            for(int jj=std::max(j-win,0); jj<=std::min(j+win,(int)msg_rcv_multi[r][l][ii].size()-1); jj++){
                if(getMultiMapValue(r,l,ii,jj)){
                    stop=true;
                    i=ii; j=jj;
                    break;
                }
            }
            if(stop)
                break;
        }
        if(!stop)
            return false;
    }
    if(!getMultiMapValue(r,ln,in,jn)){
        bool stop=false;
        for(int ii=std::max(in-win,0); ii<=std::min(in+win,(int)msg_rcv_multi[r][ln].size()-1); ii++){
            for(int jj=std::max(jn-win,0); jj<=std::min(jn+win,(int)msg_rcv_multi[r][ln][ii].size()-1); jj++){
                if(getMultiMapValue(r,ln,ii,jj)){
                    stop=true;
                    in=ii; jn=jj;
                    break;
                }
            }
            if(stop)
                break;
        }
        if(!stop)
            return false;
    }
    std::vector<std::vector<bool> > temp=msg_rcv_multi[r][l];
    for(int row=std::max(i-3*win,0);row<=std::min(i+3*win,(int)temp.size());row++){
        for(int col=std::max(j-3*win,0);col<=std::min(j+3*win,(int)temp[row].size());col++){
            if( msg_rcv_multi[r][ln][row][col] )
                temp[row][col]=true;
        }
    }
    Apath path=Astar<float>(PointI(i,j), PointI(in,jn), temp);
    if( path.cost<=(adj*sqrt((i-in)*(i-in)+(j-jn)*(j-jn))) && path.cost>0)
        return true;
    else
        return false;
}

bool PddlGenNC::connection_ray(int i, int j, int l, int in, int jn, int r_e, int r_v, bool strict){
    if(!getMultiMapValue(r_v,l,in,jn)){
        return false;
    }
    if(!getMultiMapValue(r_e,l,i,j)){
        bool stop=false;
        for(int ii=std::max(i-win,0); ii<=std::min(i+win,(int)msg_rcv_multi[r_e][l].size()-1); ii++){
            for(int jj=std::max(j-win,0); jj<=std::min(j+win,(int)msg_rcv_multi[r_e][l][ii].size()-1); jj++){
                if(getMultiMapValue(r_e,l,ii,jj)){
                    stop=true;
                    i=ii; j=jj;
                    break;
                }
            }
            if(stop)
                break;
        }
        if(!stop)
            return false;
    }
    int rr=r_e%nrobots;
    if(strict && !getStructCenteredValue(rr,l,(in-i),(jn-j)))
        return false;
    if(raytracing(or_map, i, j, in, jn, true))
        return true;
    else
        return false;
}

bool PddlGenNC::allMapsReceived(void){
    for(unsigned int i=0; i<map_rcv.size(); i++){
        if(!map_rcv[i] || (msg_rcv[i].size()==0) || (msg_rcv[i][0].size()==0))
            return false;
    }
    return true;
}

bool PddlGenNC::allActMapsReceived(void){
    for(unsigned int i=0; i<map_rcv_act.size(); i++){
        if(!map_rcv_act[i] || (msg_rcv_act[i].rows==0) || (msg_rcv_act[i].cols==0))
            return false;
    }
    return true;
}

bool PddlGenNC::allMultiMapsReceived(void){
    for(unsigned int i=0; i<map_rcv_multi.size(); i++){
        if(!map_rcv_multi[i] || (msg_rcv_multi[i].size()==0) || (msg_rcv_multi[i][0].size()==0) || (msg_rcv_multi[i][0][0].size()==0))
            return false;
    }
    return true;
}


bool PddlGenNC::allMultiStrMapsReceived(void){
    for(unsigned int i=0; i<map_rcv_multi_str.size(); i++){
        if(!map_rcv_multi_str[i] || (msg_rcv_multi_str[i].size()==0) || (msg_rcv_multi_str[i][0].size()==0) || (msg_rcv_multi_str[i][0][0].size()==0))
            return false;
    }
    return true;
}


bool PddlGenNC::allRobCenterReceived(void){
    for(unsigned int i=0; i<rob_center.size(); i++){
        if((rob_center[i].x<0) || (rob_center[i].y<0))
            return false;
    }
    return true;
}

bool PddlGenNC::validWaypoint(unsigned int i, unsigned int j){
    for(int rr=0; rr<nrobots; rr++){
        //if(getMapValue(rr,i,j))
        if(getMapValue(nrobots*O_MAP,i,j))
            return true;
    }
    return false;
}

bool PddlGenNC::feasibleWaypoint(unsigned int i, unsigned int j){
    for(int rr=0; rr<nrobots; rr++){
        if(getMapValue(PA_MAP*nrobots+rr,i,j))
            return true;
    }
    return false;
}

bool PddlGenNC::run(void){
    if( allMapsReceived() && allActMapsReceived() && allMultiMapsReceived() && allMultiStrMapsReceived() && allRobCenterReceived()){
        std::vector<cv::Point3i> pos(nrobots);
        std::vector<cv::Point2i> pos2(nrobots);
        for(int i=0;i<nrobots;i++){
            tf::StampedTransform transform;
            std::string frame= "/robot_"+std::to_string(i)+"/base_link";
            try{
                pos_listener.lookupTransform("/map",frame, ros::Time(0), transform);
            }
            catch (tf::TransformException ex){
              ROS_INFO("%s",ex.what());
              return false;
            }
            geometry_msgs::Point pt;
            pt.x=transform.getOrigin().x();
            pt.y=transform.getOrigin().y();
            convertWtf2I( pt, res, pos2[i] );
            pos[i].x=pos2[i].x;
            pos[i].y=pos2[i].y;

            pt.z=tf::getYaw(transform.getRotation())/2/PI*360;
            pt.z=boundAngleD(pt.z);
            pos[i].z=angleD2I(pt.z, msg_rcv_multi[i].size());
            //std::cout<<pos[i].x<<" "<<pos[i].y<<" "<<pos[i].z<<std::endl;
        }
        for(int rr=0;rr<nrobots;rr++){
            if(!getMultiMapValue(ME_MAP*nrobots+rr,pos[rr].z,pos[rr].x,pos[rr].y))
                return false;
        }
        unsigned int number_layers=msg_rcv_multi[0].size();
        std::vector<std::vector<long int> > waypoints(msg_rcv[0].size(), std::vector<long int>(msg_rcv[0][0].size(),-1));
        std::vector<cv::Point2i> names(0);
        wp_n2i.assign(waypoints.size()/jump+1, std::vector<long int>(waypoints[0].size()/jump+1,-1));
        long int count=0;
        //int jump=min(r0s,r1s)/2;
        for(unsigned int i=0;i<waypoints.size();i=i+jump){
            unsigned int in=i/jump;
            for(unsigned int j=0;j<waypoints[i].size();j=j+jump){
                unsigned int jn=j/jump;
                if( validWaypoint(i,j) ){
                    waypoints[i][j]=count;
                    names.push_back(cv::Point(in,jn));
                    wp_n2i[in][jn]=count;
                    wps.push_back(cv::Point(i,j));
                    count++;
                }
            }
        }
        std::vector<std::vector<std::vector<long int> > > waypoints3D(number_layers,std::vector<std::vector<long int> >(msg_rcv[0].size(), std::vector<long int>(msg_rcv[0][0].size(),-1)));
        std::vector<cv::Point3i> names3D(0);
        wp_n2i3D.assign(number_layers,std::vector<std::vector<long int> >(waypoints.size()/jump+1, std::vector<long int>(waypoints[0].size()/jump+1,-1)));
        long int count3D=0;
        for(unsigned int ll=0; ll<number_layers;ll++){
            for(unsigned int i=0;i<waypoints.size();i=i+jump){
                unsigned int in=i/jump;
                for(unsigned int j=0;j<waypoints[i].size();j=j+jump){
                    unsigned int jn=j/jump;
                    if( validWaypoint(i,j) ){
                        waypoints3D[ll][i][j]=count3D;
                        names3D.push_back(cv::Point3i(in,jn,ll));
                        wp_n2i3D[ll][in][jn]=count3D;
                        wps3D.push_back(cv::Point3i(i,j,ll));
                        count3D++;
                    }
                }
            }
        }
        graph.assign(nrobots, std::vector<std::vector<bool> >(count3D, std::vector<bool>(count3D,false)));
        actuable.assign(nrobots, std::vector<std::vector<bool> >(count3D, std::vector<bool>(count,false)));
        std::vector<std::vector<bool> > visibleT(nrobots, std::vector<bool>(count, false));
        std::vector<std::vector<bool> > visibleTR(nrobots, std::vector<bool>(count, false));
        std::vector<std::vector<bool> > connectTR(nrobots, std::vector<bool>(count3D, false));
        for(unsigned int ll=0; ll<waypoints3D.size();ll++){
            for(unsigned int i=0;i<waypoints3D[ll].size();i++){
                for(unsigned int j=0;j<waypoints3D[ll][i].size();j++){
                    if(waypoints3D[ll][i][j]>=0){
                        int imin=(int)round(std::max((float)i-(1.5*((float)jump)),0.0));
                        int jmin=(int)round(std::max((float)j-(1.5*((float)jump)),0.0));
                        int imax=(int)round(std::min((float)i+(1.5*((float)jump)),(double)waypoints3D[ll].size()-1));
                        int jmax=(int)round(std::min((float)j+(1.5*((float)jump)),(double)waypoints3D[ll][i].size()-1));
                        int la=decAngle(ll,number_layers);
                        int ua=incAngle(ll,number_layers);
                        for(unsigned int run=0; run<3; run++){
                            int angle;
                            if(run==0)
                                angle=ll;
                            if(run==1)
                                angle=la;
                            if(run==2)
                                angle=ua;
                            else
                                angle=ll;
                            for(int in=imin;in<=imax;in++){
                                for(int jn=jmin;jn<=jmax;jn++){
                                    if(waypoints3D[angle][in][jn]>=0){
                                        //if((in==(int)i || jn==(int)j) && (in!=(int)i || jn!=(int)j) ){
                                        if( std::max(abs(in-(int)i),abs(jn-(int)j))<=jump && (in!=(int)i || jn!=(int)j || angle!=(int)ll) ){
                                            for(int rr=0;rr<nrobots;rr++){
                                                if(connection((int)i,(int)j,(int)ll,in,jn,angle,ME_MAP*nrobots+rr)){
                                                    graph[rr][waypoints3D[angle][in][jn]][waypoints3D[ll][i][j]]=true;
                                                    graph[rr][waypoints3D[ll][i][j]][waypoints3D[angle][in][jn]]=true;
                                                    connectTR[rr][waypoints3D[ll][i][j]]=true;
                                                    connectTR[rr][waypoints3D[angle][in][jn]]=true;
                                                }
                                                if(connection((int)i,(int)j,(int)ll,in,jn,angle,MR_MAP*nrobots+rr)){
                                                    graph[rr][waypoints3D[angle][in][jn]][waypoints3D[ll][i][j]]=true;
                                                    graph[rr][waypoints3D[ll][i][j]][waypoints3D[angle][in][jn]]=true;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        for(int rr=0;rr<nrobots;rr++){
                            cv::Point lb=getStructLower(rr);
                            cv::Point ub=getStructUpper(rr);
                            for(int m=std::max((int)i-lb.x,0);m<=std::min((int)i+ub.x,(int)waypoints3D[ll].size()-1);m++){
                                for(int n=std::max((int)j-lb.y,0);n<=std::min((int)j+ub.y,(int)waypoints3D[ll][i].size()-1);n++){
                                    if(waypoints3D[ll][m][n]>=0 && waypoints[m][n]>=0){
                                        //if( getStructCenteredValue(rr,ll,(m-(int)i),(n-(int)j)) ){
                                            //if( getMapValue(2,i,j) && getMapValue(0,m,n) ){
                                            if(connection_ray(i,j,ll,m,n,ME_MAP*nrobots+rr,MC_MAP*nrobots+rr)){
                                                actuable[rr][waypoints3D[ll][i][j]][waypoints[m][n]]=true;
                                                visibleT[rr][waypoints[m][n]]=true;
                                            }
                                            if(connection_ray(i,j,ll,m,n,MR_MAP*nrobots+rr,MA_MAP*nrobots+rr)){
                                                actuable[rr][waypoints3D[ll][i][j]][waypoints[m][n]]=true;
                                                visibleTR[rr][waypoints[m][n]]=true;
                                            }
                                        //}
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        ROS_INFO("TEST!!!!!!");
        std::vector<std::vector<bool> > visibleTRF=visibleTR;
        std::vector<std::vector<bool> > visibleTF=visibleT;
        for(unsigned int i=0;i<waypoints.size();i++){
            for(unsigned int j=0;j<waypoints[i].size();j++){
                if(waypoints[i][j]>=0){
                    for(int rr=0;rr<nrobots;rr++){
                        if( !visibleT[rr][waypoints[i][j]] || !visibleTR[rr][waypoints[i][j]] ){
                            if(!getMapValue(PC_MAP*nrobots+rr,i,j))
                                continue;
                            float searchwin=1.0*((float)jump)+((float)std::max(msg_rcv_multi_str[rr][0].size(),msg_rcv_multi_str[rr][0][0].size()));
                            int imin=(int)round(std::max((float)i-searchwin,(float)0.0));
                            int jmin=(int)round(std::max((float)j-searchwin,(float)0.0));
                            int imax=(int)round(std::min((float)i+searchwin,(float)waypoints.size()-1));
                            int jmax=(int)round(std::min((float)j+searchwin,(float)waypoints[i].size()-1));
                            FindMin<long int, long int> wp_v, wp_vr;
                            for(int ll=0;ll<(int)number_layers;ll++){
                                for(int in=imin;in<=imax;in++){
                                    for(int jn=jmin;jn<=jmax;jn++){
                                        if(waypoints3D[ll][in][jn]>=0){
                                            //if( ((in-(int)i)*(in-(int)i)+(jn-(int)j)*(jn-(int)j))<=(searchwin*searchwin) ){
                                                if(!visibleT[rr][waypoints[i][j]] && connection_ray(in,jn,ll,(int)i,(int)j,ME_MAP*nrobots+rr,MC_MAP*nrobots+rr, false) ){
                                                    //visible[rr][waypoints[in][jn]][waypoints[i][j]]=true;
                                                    wp_v.iter((in-(int)i)*(in-(int)i)+(jn-(int)j)*(jn-(int)j), waypoints3D[ll][in][jn]);
                                                    visibleTF[rr][waypoints[i][j]]=true;
                                                }
                                                if(!visibleTR[rr][waypoints[i][j]] && connection_ray(in,jn,ll,(int)i,(int)j,MR_MAP*nrobots+rr,MA_MAP*nrobots+rr, false)){
                                                    //visible[rr][waypoints[in][jn]][waypoints[i][j]]=true;
                                                    wp_vr.iter((in-(int)i)*(in-(int)i)+(jn-(int)j)*(jn-(int)j), waypoints3D[ll][in][jn]);
                                                    visibleTRF[rr][waypoints[i][j]]=true;
                                                }
                                            //}
                                        }
                                    }
                                }
                            }
                            if(wp_v.valid()){
                                actuable[rr][wp_v.getP()][waypoints[i][j]]=true;
                            }
                            if(wp_vr.valid()){
                                actuable[rr][wp_vr.getP()][waypoints[i][j]]=true;
                            }
                        }
                    }
                }
            }
        }
        ROS_INFO("TEST1233333333!!!!!!");
        std::vector<long int> goals(0);
        std::vector<std::vector<long int> > goalsR(nrobots, std::vector<long int>(0));
        std::vector<std::vector<long int> > goalsRP(nrobots, std::vector<long int>(0));
        std::vector<std::vector<long int> > goalsRPH(nrobots, std::vector<long int>(0));
        for(int rr=0;rr<nrobots;rr++){
            goalsR[rr].clear();
            goalsRP[rr].clear();
            goalsRPH[rr].clear();
        }
        std::vector<long int> goalsUT(0);
        goalsUT.clear();
        std::vector<int> unfeasible_waypoints(count, 0);
        for(unsigned int i=0;i<waypoints.size();i++){
            for(unsigned int j=0;j<waypoints[i].size();j++){
                //if( feasibleWaypoint(i,j) ){
                    if(waypoints[i][j]>=0){
                        goals.push_back(waypoints[i][j]);

                        for(int rr=0;rr<nrobots;rr++){
                            if(!getMapValue(PA_MAP*nrobots+rr,i,j)){
                                goalsR[rr].push_back(waypoints[i][j]);
                                unfeasible_waypoints[waypoints[i][j]]++;
                            }
                            else
                            {
                                if( !visibleTRF[rr][waypoints[i][j]] ){
                                    goalsR[rr].push_back(waypoints[i][j]);
                                    unfeasible_waypoints[waypoints[i][j]]++;
                                }
                                else{
                                    goalsRP[rr].push_back(waypoints[i][j]);
                                    //goalsRPH[rr].push_back((int)round(((float)getActMapValue(rr,i,j))/((float)jump)));
                                    goalsRPH[rr].push_back((int)ceil(((float)getActMapValue(rr,i,j))/((float)jump)));
                                }
                            }
                        }
                        if(unfeasible_waypoints[waypoints[i][j]]==nrobots){
                            goalsUT.push_back(waypoints[i][j]);
                        }
                    }
                //}
            }
        }
        std::vector<long int> initials(nrobots,-1);
        //initials[0]=waypoints[pos_r0.x][pos_r0.y];
        //initials[1]=waypoints[pos_r1.x][pos_r1.y];
        std::vector<cv::Point> hlx=bf_hlx(jump+1);
        for(int rr=0;rr<nrobots;rr++){
            for(unsigned int h=0;h<hlx.size();h++){
                int px=pos[rr].x+hlx[h].x;
                int py=pos[rr].y+hlx[h].y;
                if(px>=0 && py>=0 && px<(int)waypoints.size() && py<(int)waypoints[0].size()){
                    if(waypoints3D[pos[rr].z][px][py]>=0 && valid(px,py,pos[rr].z,R_MAP*nrobots+rr)){
                        if( connectTR[rr][waypoints3D[pos[rr].z][px][py]] ){
                            initials[rr]=waypoints3D[pos[rr].z][px][py];
                            break;
                        }
                    }
                }
            }
        }
        std::stringstream strstream(""), strstreamGA(""), strstreamGAP(""), strstreamGAP_Heur(""), strstreamGA_P_Heur(""), strstreamUT("");
        write_preamble(strstream, nrobots,count, names, count3D,names3D);
        write_graph(strstream, graph, names3D);
        write_visible(strstream, actuable, names, names3D);
        write_initRobots(strstream, nrobots, initials, names3D);
        //write_goals(strstream, goals, names);//only feasible
        write_goals(strstream, count, names);//total coverage, even if not feasible
        write_goals_GA(strstreamGA, goalsR, names);//only unfeasible
        write_goals_GAP(strstreamGAP, goalsRP, names);
        write_goals_GAP_Heur(strstreamGAP_Heur, goalsRPH);
        write_goals_GA_P_Heur(strstreamGA_P_Heur, goalsRP, names, goalsRPH);
        write_goals_UT(strstreamUT, goalsUT, names);//only unfeasible
//        cout<<count<<endl;
//        for(unsigned int i=0; i<goalsR.size();i++)
//        {
//            cout<<count-goalsR[i].size()<<" "<<((float)(goalsR[i].size()))/((float)count)<<endl;
//        }
        std::ofstream myfile;
        std::string pddlFolder = ros::package::getPath("map_transform").append("/pddl/problem.pddl");
        myfile.open(pddlFolder);
        if (myfile.is_open()){
            myfile << strstream.str();
            ROS_INFO("Wrote to PDDL file.");
        }
        else{
            ROS_INFO("Failed to open PDDL file.");
        }
        myfile.close();
        std::ofstream myfileGA;
        pddlFolder = ros::package::getPath("map_transform").append("/pddl/GA.pddl");
        myfileGA.open(pddlFolder);
        if (myfileGA.is_open()){
            myfileGA << strstreamGA.str();
            ROS_INFO("Wrote to GA file.");
        }
        else{
            ROS_INFO("Failed to open GA file.");
        }
        myfileGA.close();

        std::ofstream myfileGAP;
        pddlFolder = ros::package::getPath("map_transform").append("/pddl/GAP.pddl");
        myfileGAP.open(pddlFolder);
        if (myfileGAP.is_open()){
            myfileGAP << strstreamGAP.str();
            ROS_INFO("Wrote to GAP file.");
        }
        else{
            ROS_INFO("Failed to open GAP file.");
        }
        myfileGAP.close();

        std::ofstream myfileGAP_Heur;
        pddlFolder = ros::package::getPath("map_transform").append("/pddl/Heur.pddl");
        myfileGAP_Heur.open(pddlFolder);
        if (myfileGAP_Heur.is_open()){
            myfileGAP_Heur << strstreamGAP_Heur.str();
            ROS_INFO("Wrote to Heur file.");
        }
        else{
            ROS_INFO("Failed to open Heur file.");
        }
        myfileGAP_Heur.close();

        std::ofstream myfileGA_P_Heur;
        pddlFolder = ros::package::getPath("map_transform").append("/pddl/input.lisp");
        myfileGA_P_Heur.open(pddlFolder);
        if (myfileGA_P_Heur.is_open()){
            myfileGA_P_Heur << strstreamGA_P_Heur.str();
            ROS_INFO("Wrote to Lisp Input file.");
        }
        else{
            ROS_INFO("Failed to open List Input file.");
        }
        myfileGA_P_Heur.close();

        std::ofstream myfileUT;
        pddlFolder = ros::package::getPath("map_transform").append("/pddl/unfeasible.lisp");
        myfileUT.open(pddlFolder);
        if (myfileUT.is_open()){
            myfileUT << strstreamUT.str();
            ROS_INFO("Wrote to Lisp Unfeasible file.");
        }
        else{
            ROS_INFO("Failed to open List Unfeasible file.");
        }
        myfileUT.close();

        return true;
    }
    return false;
}

enum ActionID {navigate, look, null};

struct Action{
    ActionID actID;
    int robID;
    std::vector<cv::Point2i> wp;
    int size_x, size_y, rob_max;
    Action(int sx,int sy, int rm): actID(null), robID(-1), size_x(sx), size_y(sy), rob_max(rm){
        wp.assign(2,cv::Point2i(-1,-1));
        rob_max=rm;
        size_x=sx;
        size_y=sy;
    }
    bool valid(){
        return (actID!=null) && (wp[0].x>=0) && (wp[0].y>=0) && (wp[1].x>=0) && (wp[1].y>=0) &&
                (wp[0].x<size_x) && (wp[0].y<size_y) && (wp[1].x<size_x) && (wp[1].y<size_y) && (robID>0) && (robID<=rob_max);
    }
};

bool procAct(std::string elem, Action& act){
    transform(elem.begin(), elem.end(), elem.begin(), ::tolower);
    if (elem.find("navigate") != std::string::npos) {
        act.actID=navigate;
        return true;
    }
    if (elem.find("look-at-point") != std::string::npos) {
        act.actID=look;
        return true;
    }
    return false;
}

bool procRob(std::string elem, Action& act){
    transform(elem.begin(), elem.end(), elem.begin(), ::tolower);
    size_t pos=elem.find("robot");
    if (pos!= std::string::npos) {
        act.robID=stoi(elem.substr(pos+5));
        return true;
    }
    return false;
}

bool procWP(std::string elem, Action& act, unsigned int ind){
    if(ind==0 || ind==1)
    {
        transform(elem.begin(), elem.end(), elem.begin(), ::tolower);
        size_t pos=elem.find("waypoint");
        if (pos!= std::string::npos) {
            act.wp[ind].x=stoi(elem.substr(pos+8));
            size_t pos=elem.find("_");
            if (pos!= std::string::npos) {
                act.wp[ind].y=stoi(elem.substr(pos+1));
                return true;
            }
        }
    }
    return false;
}

bool PddlGenNC::procPaths(std::vector<std::string> input){
    for(int rr=0;rr<nrobots;rr++){
        paths[rr].poses.clear();
    }
    geometry_msgs::PoseStamped pw;
    pw.header.frame_id   = "/map";
    pw.header.stamp =  ros::Time::now();
    pw.pose.orientation.w=1;

    std::vector<std::vector<long int> > paths_robots(nrobots);
    for(unsigned int i=0; i<input.size(); i++){
        std::istringstream iss(input[i]);
        std::string elem;
        Action act(wp_n2i.size(), wp_n2i[0].size(), nrobots);
        while(iss>>elem){
            if( procAct(elem, act) ){
                iss>>elem;
                if( procRob(elem,act) ){
                    iss>>elem;
                    if( procWP(elem,act,0) ){
                        iss>>elem;
                        if( procWP(elem,act,1) ){
                            if( act.valid() && act.actID==navigate ){
                                //cout<<"navigate: "<<act.robID<<" "<<act.wp[0].x<<" "<<act.wp[0].y<<" "<<act.wp[1].x<<" "<<act.wp[1].y<<endl;
                                //cout<<"navigate: "<<act.robID<<" "<<wp_n2i[act.wp[0].x][act.wp[0].y]<<" "<<wp_n2i[act.wp[1].x][act.wp[1].y]<<endl;
                                if(paths_robots[act.robID-1].size()==0)
                                {
                                    paths_robots[act.robID-1].push_back(wp_n2i[act.wp[0].x][act.wp[0].y]);
                                    paths_robots[act.robID-1].push_back(wp_n2i[act.wp[1].x][act.wp[1].y]);
                                }
                                else
                                    paths_robots[act.robID-1].push_back(wp_n2i[act.wp[1].x][act.wp[1].y]);
                            }
                            if( act.valid() && act.actID==look ){
                                //cout<<"look: "<<act.robID<<" "<<wp_n2i[act.wp[0].x][act.wp[0].y]<<" "<<wp_n2i[act.wp[1].x][act.wp[1].y]<<endl;
                            }
                        }
                    }
                }
            }
        }
    }
    for(int rr=0;rr<nrobots;rr++)
        if(paths_robots[rr].size()!=0)
            for(unsigned int p_i=0;p_i<paths_robots[rr].size();p_i++)
            {
                pw.pose.position=convertI2W(wps[paths_robots[rr][p_i]], res);

                paths[rr].poses.push_back(pw);
            }

    return true;
}

bool suc=false;

void PddlGenNC::readOutput(void){
    if(!output && suc){
        std::ifstream myfile;
        std::string pddlFolder = ros::package::getPath("map_transform").append("/pddl/paths.txt");
        myfile.open(pddlFolder);
        if (myfile.is_open()){
            std::string line;
            std::vector<std::string> input;
            while (getline(myfile, line)) {
                input.push_back(line);
            }
            output=procPaths(input);
            if(output)
                ROS_INFO("Successfully read Paths file.");
        }
        else{
            ROS_INFO("Failed to open Paths file.");
        }
        myfile.close();
    }
}

void PddlGenNC::publish(void){
    if(output){
        for(int rr=0;rr<nrobots;rr++){
            paths[rr].header.frame_id = "/map";
            paths[rr].header.stamp =  ros::Time::now();
            pubs[rr].publish(paths[rr]);
        }
    }
    visualization_msgs::MarkerArray points;
    visualization_msgs::Marker point;
    if(wps.size()>0){
        point.header.frame_id = "/map";
        point.header.stamp =  ros::Time::now();
        point.ns = "points_and_lines";
        point.action = visualization_msgs::Marker::ADD;
        point.pose.orientation.w = 1.0;
        point.type = visualization_msgs::Marker::SPHERE;
        point.scale.x = 0.1;
        point.scale.y = 0.1;
        point.scale.z = 0.02;
        point.color.g = 1.0f;
        point.color.a = 1.0;
        point.lifetime = ros::Duration(10000000);
        for (uint32_t i = 0; i < wps.size(); ++i){
            point.pose.position=convertI2W(wps[i], res);
            point.id = i;
            points.markers.push_back(point);
        }
        pub_mar[0].publish(points);
        points.markers.clear();
        point.type=visualization_msgs::Marker::LINE_STRIP;
        point.scale.x = 0.03;
        point.scale.y = 0.02;
        point.scale.z = 0.001;
        geometry_msgs::Point p;
        p.x=0.0f; p.y=0.0f; p.z=0.0f;
        point.pose.position=p;
        for(int rr=0;rr<nrobots;rr++){
            point.color.b = 1.0f;
            point.color.r = 0.0f;
            point.color.g = 1.0f;
            for(unsigned int i=0;i<graph[rr].size();i++){
                for(unsigned int j=0;j<graph[rr][i].size();j++){
                    if( graph[rr][i][j] ){
                        p=convertI2W(wps[i], res);
                        point.points.clear();
                        point.points.push_back(p);
                        p=convertI2W(wps[j], res);
                        point.points.push_back(p);
                        point.id = (i)*wps.size()+j;
                        points.markers.push_back(point);
                    }
                }
            }
            pub_mar[1+2*rr].publish(points);
            points.markers.clear();

            point.color.r = 1.0f;
            point.color.b = 0.0f;
            point.color.g = 1.0f;
            for(unsigned int i=0;i<actuable[rr].size();i++){
                for(unsigned int j=0;j<actuable[rr][i].size();j++){
                    if( actuable[rr][i][j] ){
                        p=convertI2W(wps[i], res);
                        point.points.clear();
                        point.points.push_back(p);
                        p=convertI2W(wps[j], res);
                        point.points.push_back(p);
                        point.id = (i)*wps.size()+j;
                        points.markers.push_back(point);
                    }
                }
            }
            pub_mar[1+2*rr+1].publish(points);
            points.markers.clear();
        }
    }
    else{
        visualization_msgs::MarkerArray points;
        visualization_msgs::Marker point;
        point.header.frame_id = "/map";
        point.header.stamp =  ros::Time::now();
        point.ns = "points_and_lines";
        point.action = 3;
        //point.id = goals.size();
        points.markers.push_back(point);
        //point.id = 0;
        //points.markers.push_back(point);

        pub_mar[0].publish(points);
        for(int rr=0;rr<nrobots;rr++){
            pub_mar[1+2*rr].publish(points);
            pub_mar[1+2*rr+1].publish(points);
        }
    }
}

// Replacement SIGINT handler
void HandlerStop(int){
    if(!suc)
        ROS_INFO("Failed generation of pddl.");
    ros::shutdown();
    exit(-1);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "pddl", ros::init_options::NoSigintHandler);
  signal(SIGINT, HandlerStop);
  ros::NodeHandle nh("~");
  PddlGenNC pddl(nh);
  ros::Rate loop_rate(10);
  while (ros::ok()){
    ros::spinOnce();
    if(!suc){
        if(pddl.run()){
            suc=true;
            ROS_INFO("Successful generation of pddl.");
            //pddl.publish();
            //break;
        }
    }
    pddl.readOutput();
    pddl.publish();
    loop_rate.sleep();
  }
  if(!suc)
      ROS_INFO("Failed generation of pddl!");
  else
      //ROS_INFO("Successful generation of pddl.");
  ros::shutdown();
  return 0;
}