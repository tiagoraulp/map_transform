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

using namespace std;

class PddlGen{
private:
    ros::NodeHandle nh_;
    vector<ros::Subscriber> subs;
    vector<ros::Subscriber> subs_act;
    vector<ros::Publisher> pub_mar;
    vector<ros::Publisher> pubs;
    vector<nav_msgs::Path> paths;
    bool output;
    vector<vector<vector<bool> > >  msg_rcv;
    vector<cv::Mat_<int> >  msg_rcv_act;
    vector<int> countM;
    vector<bool> map_rcv;
    vector<bool> map_rcv_act;
    float res;
    int width,height;
    cv::Mat or_map;
    int jump, nrobots;
    vector<int> rs;
    tf::TransformListener pos_listener;
    vector<cv::Point2i> wps;
    vector<vector<vector<bool> > > graph;
    vector<vector<vector<bool> > > visible;
    vector<vector<long int> > wp_n2i;
    void rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg, int index);
    void rcv_map1(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map2(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map3(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map4(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map5(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map6(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map7(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map8(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map9(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map10(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map11(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map12(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map13(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map14(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map15(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map16(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map17(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map18(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map19(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map20(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map21(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map_act(const std_msgs::Int16MultiArray::ConstPtr& msg, int index);
    void rcv_map1_act(const std_msgs::Int16MultiArray::ConstPtr& msg);
    void rcv_map2_act(const std_msgs::Int16MultiArray::ConstPtr& msg);
    void rcv_map3_act(const std_msgs::Int16MultiArray::ConstPtr& msg);
    void rcv_map4_act(const std_msgs::Int16MultiArray::ConstPtr& msg);
    void rcv_map5_act(const std_msgs::Int16MultiArray::ConstPtr& msg);
    bool getMapValue(int n, int i, int j);
    int getActMapValue(int n, int i, int j);
    cv::Point2i convertW2I(geometry_msgs::Point p);
    geometry_msgs::Point convertI2W(cv::Point2i p);
    bool valid(int i, int j, int r);
    bool connection(int i, int j, int in, int jn, int r);
    bool connection_ray(int i, int j, int in, int jn, int r_e, int r_v, bool strict=true);
    bool procPaths(vector<string> file);
    bool allMapsReceived(void);
    bool allActMapsReceived(void);
    bool validWaypoint(unsigned int i, unsigned int j);
    bool feasibleWaypoint(unsigned int i, unsigned int j);
public:
    PddlGen(ros::NodeHandle nh): nh_(nh){
        nh_.param("nrobots", nrobots, 2);
        countM.assign(4*nrobots+1,0);
        map_rcv.assign(4*nrobots+1,false);
        msg_rcv.resize(4*nrobots+1);
        map_rcv_act.assign(nrobots,false);
        msg_rcv_act.resize(nrobots);
        wps.clear();
        graph.clear();
        visible.clear();
        vector<void (PddlGen::*)(const nav_msgs::OccupancyGrid::ConstPtr& msg)> v_f;
        v_f.push_back(&PddlGen::rcv_map1);
        v_f.push_back(&PddlGen::rcv_map2);
        v_f.push_back(&PddlGen::rcv_map3);
        v_f.push_back(&PddlGen::rcv_map4);
        v_f.push_back(&PddlGen::rcv_map5);
        v_f.push_back(&PddlGen::rcv_map6);
        v_f.push_back(&PddlGen::rcv_map7);
        v_f.push_back(&PddlGen::rcv_map8);
        v_f.push_back(&PddlGen::rcv_map9);
        v_f.push_back(&PddlGen::rcv_map10);
        v_f.push_back(&PddlGen::rcv_map11);
        v_f.push_back(&PddlGen::rcv_map12);
        v_f.push_back(&PddlGen::rcv_map13);
        v_f.push_back(&PddlGen::rcv_map14);
        v_f.push_back(&PddlGen::rcv_map15);
        v_f.push_back(&PddlGen::rcv_map16);
        v_f.push_back(&PddlGen::rcv_map17);
        v_f.push_back(&PddlGen::rcv_map18);
        v_f.push_back(&PddlGen::rcv_map19);
        v_f.push_back(&PddlGen::rcv_map20);
        subs.resize(nrobots*4+1);
        for(unsigned int i=0;i<4;i++){
            char top;
            switch (i) {
            case 0:
                top='c';
                break;
            case 1:
                top='e';
                break;
            case 2:
                top='v';
                break;
            case 3:
                top='r';
                break;
            default:
                top='c';
                break;
            }
            for(int j=0;j<nrobots;j++){
                string topic="/robot_"+to_string(j)+"/"+top+"_map";
                subs[i*nrobots+j]=nh_.subscribe(topic, 1, v_f[i*nrobots+j], this);
            }
        }
        subs[subs.size()-1] = nh_.subscribe("/map", 1, &PddlGen::rcv_map21, this);


        vector<void (PddlGen::*)(const std_msgs::Int16MultiArray::ConstPtr& msg)> v_f_act;
        v_f_act.push_back(&PddlGen::rcv_map1_act);
        v_f_act.push_back(&PddlGen::rcv_map2_act);
        v_f_act.push_back(&PddlGen::rcv_map3_act);
        v_f_act.push_back(&PddlGen::rcv_map4_act);
        v_f_act.push_back(&PddlGen::rcv_map5_act);
        subs_act.resize(nrobots);
        for(int j=0;j<nrobots;j++){
            string topic="/robot_"+to_string(j)+"/"+"a_map";
            subs_act[j]=nh_.subscribe(topic, 1, v_f_act[j], this);
        }

        output=false;
        pubs.resize(nrobots);
        for(int j=0;j<nrobots;j++){
            string topic="path"+to_string(j);
            pubs[j]=nh_.advertise<nav_msgs::Path>(topic, 1,true);
        }
        pub_mar.resize(nrobots*2+1);
        pub_mar[0] = nh_.advertise<visualization_msgs::MarkerArray>("waypoints", 1,true);
        for(int j=0;j<nrobots;j++){
            string topic="connected_"+to_string(j);
            pub_mar[1+2*j]=nh_.advertise<visualization_msgs::MarkerArray>(topic, 1,true);
            topic="visible_"+to_string(j);
            pub_mar[1+2*j+1]=nh_.advertise<visualization_msgs::MarkerArray>(topic, 1,true);
        }
        rs.resize(nrobots);
        for(int j=0;j<nrobots;j++){
            string param="/robot_"+to_string(j)+"/visibility/infl";
            nh_.param(param, rs[j], 5);
        }
        paths.resize(nrobots);
        nh_.param("jump",jump, 5);
    }
    bool run(void);
    void readOutput(void);
    void publish(void);
};

void PddlGen::rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg, int index){
    if(index==(nrobots*4))
        or_map = cv::Mat(msg->info.width, msg->info.height, CV_8UC1);

    msg_rcv[index].assign(msg->info.width, vector<bool>(msg->info.height,false));

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

void PddlGen::rcv_map1(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, 0);
}

void PddlGen::rcv_map2(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, 1);
}

void PddlGen::rcv_map3(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, 2);
}

void PddlGen::rcv_map4(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, 3);
}

void PddlGen::rcv_map5(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, 4);
}

void PddlGen::rcv_map6(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, 5);
}

void PddlGen::rcv_map7(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, 6);
}

void PddlGen::rcv_map8(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, 7);
}

void PddlGen::rcv_map9(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, 8);
}

void PddlGen::rcv_map10(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, 9);
}

void PddlGen::rcv_map11(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, 10);
}

void PddlGen::rcv_map12(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, 11);
}

void PddlGen::rcv_map13(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, 12);
}

void PddlGen::rcv_map14(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, 13);
}

void PddlGen::rcv_map15(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, 14);
}

void PddlGen::rcv_map16(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, 15);
}

void PddlGen::rcv_map17(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, 16);
}

void PddlGen::rcv_map18(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, 17);
}

void PddlGen::rcv_map19(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, 18);
}

void PddlGen::rcv_map20(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, 19);
}

void PddlGen::rcv_map21(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg, nrobots*4);
}

void PddlGen::rcv_map_act(const std_msgs::Int16MultiArray::ConstPtr& msg, int index){
    msg_rcv_act[index]=cv::Mat_<int>::ones(msg->layout.dim[0].size, msg->layout.dim[1].size)*-1;

    for(unsigned int i=0;i<msg->layout.dim[0].size;i++){
        for(unsigned int j=0;j<msg->layout.dim[1].size;j++){
                msg_rcv_act[index](i,j)=msg->data[msg->layout.data_offset+msg->layout.dim[1].stride*i+j];
        }
    }
    map_rcv_act[index]=true;
}

void PddlGen::rcv_map1_act(const std_msgs::Int16MultiArray::ConstPtr& msg){
    rcv_map_act(msg, 0);
}

void PddlGen::rcv_map2_act(const std_msgs::Int16MultiArray::ConstPtr& msg){
    rcv_map_act(msg, 1);
}

void PddlGen::rcv_map3_act(const std_msgs::Int16MultiArray::ConstPtr& msg){
    rcv_map_act(msg, 2);
}

void PddlGen::rcv_map4_act(const std_msgs::Int16MultiArray::ConstPtr& msg){
    rcv_map_act(msg, 3);
}

void PddlGen::rcv_map5_act(const std_msgs::Int16MultiArray::ConstPtr& msg){
    rcv_map_act(msg, 4);
}


bool PddlGen::getMapValue(int n, int i, int j){
    return msg_rcv[n][i][j];
}

int PddlGen::getActMapValue(int n, int i, int j){
    return msg_rcv_act[n][i][j];
}

cv::Point2i PddlGen::convertW2I(geometry_msgs::Point p){
    cv::Point2i pf(round(p.x/res),round(p.y/res));
    return pf;
}

geometry_msgs::Point PddlGen::convertI2W(cv::Point2i p){
    geometry_msgs::Point pf;
    pf.x=p.x*res+0.5*res;
    pf.y=p.y*res+0.5*res;
    return pf;
}

string convertG2S(cv::Point2i pt){
    string str;
    str.append(to_string(pt.x)).append("_").append(to_string(pt.y));
    return str;
}

void write_preamble(stringstream & str, int nr, int nw, vector<cv::Point2i> names){
    str<<"(define (problem robotprob1) (:domain robot-building)"<<endl<<endl;
    str<<"(:objects\n\t";
    for(int i=0;i<nr;i++){
     str<<"robot"<<i+1<<" ";
    }
    str<<"- robot"<<endl<<"\t";
    for(int i=0;i<nw;i++){
        str<<"waypoint"<<convertG2S(names[i])<<" ";
    }
    str<<"- waypoint"<<endl<<")"<<endl<<endl;
}

void write_graph(stringstream & str, vector<vector<vector<bool> > > graph, vector<cv::Point2i> names){
    str<<"(:init\n";
    for(unsigned int i=0; i<graph.size(); i++)
        for(unsigned int j=0; j<graph[i].size(); j++)
            for(unsigned int k=0; k<graph[i][j].size(); k++)
                if(graph[i][j][k])
                    str<<"\t(connected robot"<<i+1<<" waypoint"<<convertG2S(names[j])<<" waypoint"<<convertG2S(names[k])<<")"<<endl;
    str<<endl;
}

void write_visible(stringstream & str, vector<vector<vector<bool> > > visible, vector<cv::Point> names){
    for(unsigned int i=0; i<visible.size(); i++)
        for(unsigned int j=0; j<visible[i].size(); j++)
            for(unsigned int k=0; k<visible[i][j].size(); k++)
                if(visible[i][j][k])
                    str<<"\t(visible robot"<<i+1<<" waypoint"<<convertG2S(names[j])<<" waypoint"<<convertG2S(names[k])<<")"<<endl;
    str<<endl;
}

void write_initRobots(stringstream & str, int nr, vector<long int> initials, vector<cv::Point2i> names){
    for(int i=0; i<nr; i++){
        if(initials[i]>=0 && (initials[i]<((long int)names.size()))){
            str<<"\t(at robot"<<i+1<< " waypoint"<<convertG2S(names[initials[i]])<<")"<<endl;
        }
        else{
            str<<"\t(at robot"<<i+1<< " waypoint"<<")"<<endl;
        }
    }
    str<<endl;

    for(int i=0; i<nr; i++)
        str<<"\t(available robot"<<i+1<<")"<<endl;
    str<<")"<<endl<<endl;
}

void write_goals(stringstream & str, vector<long int> goals, vector<cv::Point2i> names){
    str<<"(:goal (and"<<endl;
    for(unsigned int i=0;i<goals.size();i++){
        str<<"\t(visited waypoint"<<convertG2S(names[goals[i]])<<")"<<endl;
    }
    str<<"\t)\n)\n)";
}

void write_goals(stringstream & str, int goals, vector<cv::Point2i> names){
    str<<"(:goal (and"<<endl;
    for(int i=0;i<goals;i++){
        str<<"\t(visited waypoint"<<convertG2S(names[i])<<")"<<endl;
    }
    str<<"\t)\n)\n)";
}

void write_goals_UT(stringstream & str, vector<long int> goalsUT, vector<cv::Point2i> names){
    str<<"(:notgoal (and"<<endl;
    for(unsigned int i=0;i<goalsUT.size();i++){
        str<<"\t(visited waypoint"<<convertG2S(names[goalsUT[i]])<<")"<<endl;
    }
    str<<"\t)\n)";
}

void write_goals_GA(stringstream & str, vector<vector<long int> > goalsR, vector<cv::Point2i> names){
    str<<"("<<endl;
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
            str<<endl;
        }
    }
    str<<")";
}

void write_goals_GAP(stringstream & str, vector<vector<long int> > goalsRP, vector<cv::Point2i> names){
    str<<"("<<endl;
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
            str<<endl;
        }
    }
    str<<")";
}

void write_goals_GAP_Heur(stringstream & str, vector<vector<long int> > goalsRPH){
    str<<"("<<endl;
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
            str<<endl;
        }
    }
    str<<")";
}

void write_goals_GA_P_Heur(stringstream & str, vector<vector<long int> > goalsRP, vector<cv::Point2i> names, vector<vector<long int> > goalsRPH){
    if( (goalsRP.size() != goalsRPH.size()) || (goalsRP.size()==0) )
        return;
    for(unsigned int r=0; r<goalsRP.size(); r++){
        if(goalsRP[r].size()!= goalsRPH[r].size())
                return;
    }
    str<<"("<<endl;
    unsigned int nr=goalsRP.size();
    for(unsigned int r=0; r<nr; r++){
        if(goalsRP[r].size()>0){
            str<<"\t(robot"<<r+1<<" ";
            for(unsigned int i=0;i<goalsRP[r].size();i++){
                str<<"((visited waypoint"<<convertG2S(names[goalsRP[r][i]])<<") "<<goalsRPH[r][i]<<") ";
            }
            str<<")";
            str<<endl;
        }
    }
    str<<")";
}

int win=3;
float adj=1.2;


bool PddlGen::valid(int i, int j, int r){
    if(!getMapValue(r,i,j)){
        bool stop=false;
        for(int ii=max(i-win,0); ii<=min(i+win,(int)msg_rcv[r].size()-1); ii++){
            for(int jj=max(j-win,0); jj<=min(j+win,(int)msg_rcv[r][ii].size()-1); jj++){
                if(getMapValue(r,ii,jj)){
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

bool PddlGen::connection(int i, int j, int in, int jn, int r){
    if(!getMapValue(r,i,j)){
        bool stop=false;
        for(int ii=max(i-win,0); ii<=min(i+win,(int)msg_rcv[r].size()-1); ii++){
            for(int jj=max(j-win,0); jj<=min(j+win,(int)msg_rcv[r][ii].size()-1); jj++){
                if(getMapValue(r,ii,jj)){
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
    if(!getMapValue(r,in,jn)){
        bool stop=false;
        for(int ii=max(in-win,0); ii<=min(in+win,(int)msg_rcv[r].size()-1); ii++){
            for(int jj=max(jn-win,0); jj<=min(jn+win,(int)msg_rcv[r][ii].size()-1); jj++){
                if(getMapValue(r,ii,jj)){
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
    Apath path=Astar<float>(PointI(i,j), PointI(in,jn), msg_rcv[r]);
    if( path.cost<=(adj*sqrt((i-in)*(i-in)+(j-jn)*(j-jn))) && path.cost>0)
        return true;
    else
        return false;
}

bool PddlGen::connection_ray(int i, int j, int in, int jn, int r_e, int r_v, bool strict){
    if(!getMapValue(r_e,i,j)){
        bool stop=false;
        for(int ii=max(i-win,0); ii<=min(i+win,(int)msg_rcv[r_e].size()-1); ii++){
            for(int jj=max(j-win,0); jj<=min(j+win,(int)msg_rcv[r_e][ii].size()-1); jj++){
                if(getMapValue(r_e,ii,jj)){
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
    if(!getMapValue(r_v,in,jn)){
        return false;
    }
    int rr=r_e%nrobots;
    if( ((in-i)*(in-i)+(jn-j)*(jn-j))>(rs[rr]*rs[rr]) && strict)
        return false;
    if(raytracing(or_map, i, j, in, jn, true))
        return true;
    else
        return false;
}

bool PddlGen::allMapsReceived(void){
    for(unsigned int i=0; i<map_rcv.size(); i++){
        if(!map_rcv[i])
            return false;
    }
    return true;
}

bool PddlGen::allActMapsReceived(void){
    for(unsigned int i=0; i<map_rcv_act.size(); i++){
        if(!map_rcv_act[i])
            return false;
    }
    return true;
}

bool PddlGen::validWaypoint(unsigned int i, unsigned int j){
    for(int rr=0; rr<nrobots; rr++){
        //if(getMapValue(rr,i,j))
        if(getMapValue(nrobots*4,i,j))
            return true;
    }
    return false;
}

bool PddlGen::feasibleWaypoint(unsigned int i, unsigned int j){
    for(int rr=0; rr<nrobots; rr++){
        if(getMapValue(2*nrobots+rr,i,j))
            return true;
    }
    return false;
}

bool PddlGen::run(void){
    if( allMapsReceived() && allActMapsReceived()){
        vector<cv::Point2i> pos(nrobots);
        for(int i=0;i<nrobots;i++){
            tf::StampedTransform transform;
            string frame= "/robot_"+to_string(i)+"/base_link";
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
            pos[i]=convertW2I( pt );
        }
        for(int rr=0;rr<nrobots;rr++){
            if(!getMapValue(nrobots+rr,pos[rr].x,pos[rr].y))
                return false;
        }
        vector<vector<long int> > waypoints(msg_rcv[0].size(), vector<long int>(msg_rcv[0][0].size(),-1));
        vector<cv::Point2i> names(0);
        wp_n2i.assign(waypoints.size()/jump+1, vector<long int>(waypoints[0].size()/jump+1,-1));
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
        graph.assign(nrobots, vector<vector<bool> >(count, vector<bool>(count,false)));
        visible.assign(nrobots, vector<vector<bool> >(count, vector<bool>(count,false)));
        vector<vector<bool> > visibleT(nrobots, vector<bool>(count, false));
        vector<vector<bool> > visibleTR(nrobots, vector<bool>(count, false));
        vector<vector<bool> > connectTR(nrobots, vector<bool>(count, false));
        for(unsigned int i=0;i<waypoints.size();i++){
            for(unsigned int j=0;j<waypoints[i].size();j++){
                if(waypoints[i][j]>=0){
                    int imin=(int)round(max((float)i-(1.5*((float)jump)),0.0));
                    int jmin=(int)round(max((float)j-(1.5*((float)jump)),0.0));
                    int imax=(int)round(min((float)i+(1.5*((float)jump)),(double)waypoints.size()-1));
                    int jmax=(int)round(min((float)j+(1.5*((float)jump)),(double)waypoints[i].size()-1));
                    for(int in=imin;in<=imax;in++){
                        for(int jn=jmin;jn<=jmax;jn++){
                            if(waypoints[in][jn]>=0){
                                //if((in==(int)i || jn==(int)j) && (in!=(int)i || jn!=(int)j) ){
                                if( max(abs(in-(int)i),abs(jn-(int)j))<=jump && (in!=(int)i || jn!=(int)j) ){
                                    for(int rr=0;rr<nrobots;rr++){
                                        if(connection((int)i,(int)j,in,jn,nrobots+rr)){
                                            graph[rr][waypoints[in][jn]][waypoints[i][j]]=true;
                                            graph[rr][waypoints[i][j]][waypoints[in][jn]]=true;
                                            connectTR[rr][waypoints[i][j]]=true;
                                            connectTR[rr][waypoints[in][jn]]=true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    for(int rr=0;rr<nrobots;rr++){
                        for(int m=max((int)i-rs[rr]-1,0);m<=min((int)i+rs[rr]+1,(int)waypoints.size()-1);m++){
                            for(int n=max((int)j-rs[rr]-1,0);n<=min((int)j+rs[rr]+1,(int)waypoints[i].size()-1);n++){
                                if( ((m-(int)i)*(m-(int)i)+(n-(int)j)*(n-(int)j))<=(rs[rr]*rs[rr]) ){
                                    //if( getMapValue(2,i,j) && getMapValue(0,m,n) ){
                                    if(connection_ray(i,j,m,n,nrobots+rr,rr)){
                                        if(waypoints[m][n]>=0){
                                            visible[rr][waypoints[i][j]][waypoints[m][n]]=true;
                                            visibleT[rr][waypoints[m][n]]=true;
                                        }
                                    }
                                    if(connection_ray(i,j,m,n,3*nrobots+rr,2*nrobots+rr)){
                                        if(waypoints[m][n]>=0){
                                            visible[rr][waypoints[i][j]][waypoints[m][n]]=true;
                                            visibleTR[rr][waypoints[m][n]]=true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        vector<vector<bool> > visibleTRF=visibleTR;
        vector<vector<bool> > visibleTF=visibleT;
        for(unsigned int i=0;i<waypoints.size();i++){
            for(unsigned int j=0;j<waypoints[i].size();j++){
                if(waypoints[i][j]>=0){
                    for(int rr=0;rr<nrobots;rr++){
                        if( !visibleT[rr][waypoints[i][j]] || !visibleTR[rr][waypoints[i][j]] ){
                            float searchwin=1.0*((float)jump)+((float)rs[rr]);
                            int imin=(int)round(max((float)i-searchwin,(float)0.0));
                            int jmin=(int)round(max((float)j-searchwin,(float)0.0));
                            int imax=(int)round(min((float)i+searchwin,(float)waypoints.size()-1));
                            int jmax=(int)round(min((float)j+searchwin,(float)waypoints[i].size()-1));
                            FindMin<long int, long int> wp_v, wp_vr;
                            for(int in=imin;in<=imax;in++){
                                for(int jn=jmin;jn<=jmax;jn++){
                                    if(waypoints[in][jn]>=0){
                                        if( ((in-(int)i)*(in-(int)i)+(jn-(int)j)*(jn-(int)j))<=(searchwin*searchwin) ){
                                            if(connection_ray(in,jn,(int)i,(int)j,nrobots+rr,rr, false) && !visibleT[rr][waypoints[i][j]]){
                                                //visible[rr][waypoints[in][jn]][waypoints[i][j]]=true;
                                                wp_v.iter((in-(int)i)*(in-(int)i)+(jn-(int)j)*(jn-(int)j), waypoints[in][jn]);
                                                visibleTF[rr][waypoints[i][j]]=true;
                                            }
                                            if(connection_ray(in,jn,(int)i,(int)j,3*nrobots+rr,2*nrobots+rr, false) && !visibleTR[rr][waypoints[i][j]]){
                                                //visible[rr][waypoints[in][jn]][waypoints[i][j]]=true;
                                                wp_vr.iter((in-(int)i)*(in-(int)i)+(jn-(int)j)*(jn-(int)j), waypoints[in][jn]);
                                                visibleTRF[rr][waypoints[i][j]]=true;
                                            }
                                        }
                                    }
                                }
                            }
                            if(wp_v.valid()){
                                visible[rr][wp_v.getP()][waypoints[i][j]]=true;
                            }
                            if(wp_vr.valid()){
                                visible[rr][wp_vr.getP()][waypoints[i][j]]=true;
                            }
                        }
                    }
                }
            }
        }
        vector<long int> goals(0);
        vector<vector<long int> > goalsR(nrobots, vector<long int>(0));
        vector<vector<long int> > goalsRP(nrobots, vector<long int>(0));
        vector<vector<long int> > goalsRPH(nrobots, vector<long int>(0));
        for(int rr=0;rr<nrobots;rr++){
            goalsR[rr].clear();
            goalsRP[rr].clear();
            goalsRPH[rr].clear();
        }
        vector<long int> goalsUT(0);
        goalsUT.clear();
        vector<int> unfeasible_waypoints(count, 0);
        for(unsigned int i=0;i<waypoints.size();i++){
            for(unsigned int j=0;j<waypoints[i].size();j++){
                //if( feasibleWaypoint(i,j) ){
                    if(waypoints[i][j]>=0){
                        goals.push_back(waypoints[i][j]);

                        for(int rr=0;rr<nrobots;rr++){
                            if(!getMapValue(2*nrobots+rr,i,j)){
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
                                    goalsRPH[rr].push_back((int)round(((float)getActMapValue(rr,i,j))/((float)jump)));
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
        vector<long int> initials(nrobots,-1);
        //initials[0]=waypoints[pos_r0.x][pos_r0.y];
        //initials[1]=waypoints[pos_r1.x][pos_r1.y];
        vector<cv::Point> hlx=bf_hlx(jump+1);
        for(int rr=0;rr<nrobots;rr++){
            for(unsigned int h=0;h<hlx.size();h++){
                int px=pos[rr].x+hlx[h].x;
                int py=pos[rr].y+hlx[h].y;
                if(px>=0 && py>=0 && px<(int)waypoints.size() && py<(int)waypoints[0].size()){
                    if(waypoints[px][py]>=0 && valid(px,py,3*nrobots+rr)){
                        if( connectTR[rr][waypoints[px][py]] ){
                            initials[rr]=waypoints[px][py];
                            break;
                        }
                    }
                }
            }
        }
        stringstream strstream(""), strstreamGA(""), strstreamGAP(""), strstreamGAP_Heur(""), strstreamGA_P_Heur(""), strstreamUT("");
        write_preamble(strstream, nrobots,count, names);
        write_graph(strstream, graph, names);
        write_visible(strstream, visible, names);
        write_initRobots(strstream, nrobots, initials, names);
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
        ofstream myfile;
        string pddlFolder = ros::package::getPath("map_transform").append("/pddl/problem.pddl");
        myfile.open(pddlFolder);
        if (myfile.is_open()){
            myfile << strstream.str();
            ROS_INFO("Wrote to PDDL file.");
        }
        else{
            ROS_INFO("Failed to open PDDL file.");
        }
        myfile.close();
        ofstream myfileGA;
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

        ofstream myfileGAP;
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

        ofstream myfileGAP_Heur;
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

        ofstream myfileGA_P_Heur;
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

        ofstream myfileUT;
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
    vector<cv::Point2i> wp;
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

bool procAct(string elem, Action& act){
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

bool procRob(string elem, Action& act){
    transform(elem.begin(), elem.end(), elem.begin(), ::tolower);
    size_t pos=elem.find("robot");
    if (pos!= string::npos) {
        act.robID=stoi(elem.substr(pos+5));
        return true;
    }
    return false;
}

bool procWP(string elem, Action& act, unsigned int ind){
    if(ind==0 || ind==1)
    {
        transform(elem.begin(), elem.end(), elem.begin(), ::tolower);
        size_t pos=elem.find("waypoint");
        if (pos!= string::npos) {
            act.wp[ind].x=stoi(elem.substr(pos+8));
            size_t pos=elem.find("_");
            if (pos!= string::npos) {
                act.wp[ind].y=stoi(elem.substr(pos+1));
                return true;
            }
        }
    }
    return false;
}

bool PddlGen::procPaths(vector<string> input){
    for(int rr=0;rr<nrobots;rr++){
        paths[rr].poses.clear();
    }
    geometry_msgs::PoseStamped pw;
    pw.header.frame_id   = "/map";
    pw.header.stamp =  ros::Time::now();
    pw.pose.orientation.w=1;

    vector<vector<long int> > paths_robots(nrobots);
    for(unsigned int i=0; i<input.size(); i++){
        istringstream iss(input[i]);
        string elem;
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
                pw.pose.position=convertI2W(wps[paths_robots[rr][p_i]]);

                paths[rr].poses.push_back(pw);
            }

    return true;
}

bool suc=false;

void PddlGen::readOutput(void){
    if(!output && suc){
        ifstream myfile;
        string pddlFolder = ros::package::getPath("map_transform").append("/pddl/paths.txt");
        myfile.open(pddlFolder);
        if (myfile.is_open()){
            string line;
            vector<string> input;
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

void PddlGen::publish(void){
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
            point.pose.position=convertI2W(wps[i]);
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
                        p=convertI2W(wps[i]);
                        point.points.clear();
                        point.points.push_back(p);
                        p=convertI2W(wps[j]);
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
            for(unsigned int i=0;i<visible[rr].size();i++){
                for(unsigned int j=0;j<visible[rr][i].size();j++){
                    if( visible[rr][i][j] ){
                        p=convertI2W(wps[i]);
                        point.points.clear();
                        point.points.push_back(p);
                        p=convertI2W(wps[j]);
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
  PddlGen pddl(nh);
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
