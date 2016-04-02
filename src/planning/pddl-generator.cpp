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
#include "tf/transform_listener.h"
#include "visualization_msgs/MarkerArray.h"
#include "vector_utils.hpp"
#include "Astar.hpp"
#include "ray.hpp"

using namespace std;

class PddlGen{
private:
    ros::NodeHandle nh_;

    ros::Subscriber sub1, sub2, sub3, sub4, sub5, sub6, sub7, sub8, sub9;
    ros::Publisher pub_mar1, pub_mar2, pub_mar3, pub_mar4, pub_mar5;

    vector<vector<vector<bool> > >  msg_rcv;
    vector<int> countM;
    vector<bool> map_rcv;
    float res;
    int width,height;

    cv::Mat or_map;

    int r0s, r1s, jump;

    tf::TransformListener pos_listener;

    vector<cv::Point2i> wps;
    vector<vector<vector<bool> > > graph;
    vector<vector<vector<bool> > > visible;

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
    bool getMapValue(int n, int i, int j);
    cv::Point2i convertW2I(geometry_msgs::Point p);
    geometry_msgs::Point convertI2W(cv::Point2i p);
    bool connection(int i, int j, int in, int jn, int r);
    bool connection_ray(int i, int j, int in, int jn, int r_e, int r_v);
public:
    PddlGen(ros::NodeHandle nh): nh_(nh)
    {
        sub1 = nh_.subscribe("/robot_0/c_map", 1, &PddlGen::rcv_map1, this);
        sub2 = nh_.subscribe("/robot_1/c_map", 1, &PddlGen::rcv_map2, this);
        sub3 = nh_.subscribe("/robot_0/e_map", 1, &PddlGen::rcv_map3, this);
        sub4 = nh_.subscribe("/robot_1/e_map", 1, &PddlGen::rcv_map4, this);
        sub5 = nh_.subscribe("/robot_0/v_map", 1, &PddlGen::rcv_map5, this);
        sub6 = nh_.subscribe("/robot_1/v_map", 1, &PddlGen::rcv_map6, this);
        sub7 = nh_.subscribe("/robot_0/r_map", 1, &PddlGen::rcv_map7, this);
        sub8 = nh_.subscribe("/robot_1/r_map", 1, &PddlGen::rcv_map8, this);
        sub9 = nh_.subscribe("/map", 1, &PddlGen::rcv_map9, this);
        countM.assign(9,0);
        map_rcv.assign(9,false);
        msg_rcv.resize(9);
        wps.clear();
        graph.clear();
        visible.clear();

        pub_mar1 = nh_.advertise<visualization_msgs::MarkerArray>("waypoints", 1,true);
        pub_mar2 = nh_.advertise<visualization_msgs::MarkerArray>("connected_0", 1,true);
        pub_mar3 = nh_.advertise<visualization_msgs::MarkerArray>("visible_0", 1,true);
        pub_mar4 = nh_.advertise<visualization_msgs::MarkerArray>("connected_1", 1,true);
        pub_mar5 = nh_.advertise<visualization_msgs::MarkerArray>("visible_1", 1,true);

        nh_.param("/robot_0/visibility/infl", r0s, 5);
        nh_.param("/robot_1/visibility/infl", r1s, 5);
        nh_.param("jump",jump, 5);
    }

    bool plan(void);
    void publish(void);
};

void PddlGen::rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg, int index)
{
    if(index==8)
        or_map = cv::Mat(msg->info.width, msg->info.height, CV_8UC1);

    msg_rcv[index].assign(msg->info.width, vector<bool>(msg->info.height,false));

    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
            if(*mapDataIterC == 0)
            {
                msg_rcv[index][j][i]=true;
                if(index==8)
                    or_map.at<uchar>(j,i) = 255;
            }
            else
            {
                msg_rcv[index][j][i]=false;
                if(index==8)
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

void PddlGen::rcv_map1(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    rcv_map(msg, 0);
}

void PddlGen::rcv_map2(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    rcv_map(msg, 1);
}

void PddlGen::rcv_map3(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    rcv_map(msg, 2);
}

void PddlGen::rcv_map4(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    rcv_map(msg, 3);
}

void PddlGen::rcv_map5(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    rcv_map(msg, 4);
}

void PddlGen::rcv_map6(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    rcv_map(msg, 5);
}

void PddlGen::rcv_map7(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    rcv_map(msg, 6);
}

void PddlGen::rcv_map8(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    rcv_map(msg, 7);
}

void PddlGen::rcv_map9(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    rcv_map(msg, 8);
}

bool PddlGen::getMapValue(int n, int i, int j)
{
    return msg_rcv[n][i][j];
}

cv::Point2i PddlGen::convertW2I(geometry_msgs::Point p)
{
    cv::Point2i pf(round(p.x/res),round(p.y/res));
    return pf;
}

geometry_msgs::Point PddlGen::convertI2W(cv::Point2i p)
{
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

void write_preamble(stringstream & str, int nr, int nw, vector<cv::Point2i> names)
{
    str<<"(define (problem robotprob1) (:domain robot-building)"<<endl<<endl;
    str<<"(:objects\n\t";
    for(int i=0;i<nr;i++)
    {
     str<<"robot"<<i+1<<" ";
    }
    str<<"- robot"<<endl<<"\t";
    for(int i=0;i<nw;i++)
    {
        str<<"waypoint"<<convertG2S(names[i])<<" ";
    }
    str<<"- waypoint"<<endl<<")"<<endl<<endl;
}



void write_graph(stringstream & str, vector<vector<vector<bool> > > graph, vector<cv::Point2i> names)
{
    str<<"(:init\n";

    for(unsigned int i=0; i<graph.size(); i++)
        for(unsigned int j=0; j<graph[i].size(); j++)
            for(unsigned int k=0; k<graph[i][j].size(); k++)
                if(graph[i][j][k])
                    str<<"\t(connected robot"<<i+1<<" waypoint"<<convertG2S(names[j])<<" waypoint"<<convertG2S(names[k])<<")"<<endl;

    str<<endl;
}

void write_visible(stringstream & str, vector<vector<vector<bool> > > visible, vector<cv::Point> names)
{
    for(unsigned int i=0; i<visible.size(); i++)
        for(unsigned int j=0; j<visible[i].size(); j++)
            for(unsigned int k=0; k<visible[i][j].size(); k++)
                if(visible[i][j][k])
                    str<<"\t(visible robot"<<i+1<<" waypoint"<<convertG2S(names[j])<<" waypoint"<<convertG2S(names[k])<<")"<<endl;

    str<<endl;
}

void write_initRobots(stringstream & str, int nr, vector<long int> initials, vector<cv::Point2i> names)
{
    for(int i=0; i<nr; i++)
        str<<"\t(at robot"<<i+1<< " waypoint"<<convertG2S(names[initials[i]])<<")"<<endl;

    str<<endl;

    for(int i=0; i<nr; i++)
        str<<"\t(available robot"<<i+1<<")"<<endl;

    str<<")"<<endl<<endl;
}


void write_goals(stringstream & str, vector<int> goals, vector<cv::Point2i> names)
{
    str<<"(:goal (and"<<endl;
    for(unsigned int i=0;i<goals.size();i++)
    {
        str<<"\t(visited waypoint"<<convertG2S(names[goals[i]])<<")"<<endl;
    }
    str<<"\t)\n)\n)";
}
void write_goals(stringstream & str, int goals, vector<cv::Point2i> names)
{
    str<<"(:goal (and"<<endl;
    for(int i=0;i<goals;i++)
    {
        str<<"\t(visited waypoint"<<convertG2S(names[i])<<")"<<endl;
    }
    str<<"\t)\n)\n)";
}

int win=3;
float adj=1.2;

bool PddlGen::connection(int i, int j, int in, int jn, int r){
//    int imin=min(i,in);
//    int imax=max(i,in);

//    int jmin=min(j,jn);
//    int jmax=max(j,jn);

//    for(int ii=imin;ii<=imax;ii++){
//        for(int jj=jmin;jj<=jmax;jj++){
//            if(!getMapValue(r,ii,jj))
//                return false;
//        }
//    }

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

bool PddlGen::connection_ray(int i, int j, int in, int jn, int r_e, int r_v){
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

    if(raytracing(or_map, i, j, in, jn, true))
        return true;
    else
        return false;
}

bool PddlGen::plan(void){
    if(map_rcv[0] && map_rcv[1] && map_rcv[2] && map_rcv[3] && map_rcv[4] && map_rcv[5] && map_rcv[6] && map_rcv[7] && map_rcv[8]){
        tf::StampedTransform transform;
        try{
            pos_listener.lookupTransform("/map", "/robot_0/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_INFO("%s",ex.what());
          return false;
        }
        geometry_msgs::Point pt;
        pt.x=transform.getOrigin().x();
        pt.y=transform.getOrigin().y();
        cv::Point2i pos_r0=convertW2I( pt );

        try{
            pos_listener.lookupTransform("/map", "/robot_1/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_INFO("%s",ex.what());
          return false;
        }
        pt.x=transform.getOrigin().x();
        pt.y=transform.getOrigin().y();
        cv::Point2i pos_r1=convertW2I( pt );

        if(!getMapValue(2,pos_r0.x,pos_r0.y))
            return false;
        if(!getMapValue(3,pos_r1.x,pos_r1.y))
            return false;


        vector<vector<long int> > waypoints(msg_rcv[0].size(), vector<long int>(msg_rcv[0][0].size(),-1));
        vector<cv::Point2i> names(0);

        long int count=0;

        //int jump=min(r0s,r1s)/2;
        //int jump=15;

        for(unsigned int i=0;i<waypoints.size();i=i+jump){
            unsigned int in=i/jump;
            for(unsigned int j=0;j<waypoints[i].size();j=j+jump){
                unsigned int jn=j/jump;
                if( getMapValue(0,i,j) || getMapValue(1,i,j) ){
                    waypoints[i][j]=count;
                    names.push_back(cv::Point(in,jn));
                    wps.push_back(cv::Point(i,j));
                    count++;
                }
            }
        }

        graph.assign(2, vector<vector<bool> >(count, vector<bool>(count,false)));
        visible.assign(2, vector<vector<bool> >(count, vector<bool>(count,false)));

        vector<vector<bool> > visibleT(2, vector<bool>(count, false));
        vector<vector<bool> > visibleTR(2, vector<bool>(count, false));

        vector<vector<bool> > connectTR(2, vector<bool>(count, false));

        for(unsigned int i=0;i<waypoints.size();i++){
            for(unsigned int j=0;j<waypoints[i].size();j++){
                if(waypoints[i][j]>=0){
                    int imin=(int)round(max((float)i-(1.5*((float)jump)),0.0));
                    int jmin=(int)round(max((float)j-(1.5*((float)jump)),0.0));
                    int imax=(int)round(min((float)i+(1.5*((float)jump)),(double)waypoints[i].size()-1));
                    int jmax=(int)round(min((float)j+(1.5*((float)jump)),(double)waypoints[i].size()-1));
                    for(int in=imin;in<=imax;in++){
                        for(int jn=jmin;jn<=jmax;jn++){
                            if(waypoints[in][jn]>=0){
                                //if((in==(int)i || jn==(int)j) && (in!=(int)i || jn!=(int)j) ){
                                if( max(abs(in-(int)i),abs(jn-(int)j))<=jump && (in!=(int)i || jn!=(int)j) ){
                                    if(connection((int)i,(int)j,in,jn,2)){
                                        graph[0][waypoints[in][jn]][waypoints[i][j]]=true;
                                        graph[0][waypoints[i][j]][waypoints[in][jn]]=true;
                                        connectTR[0][waypoints[i][j]]=true;
                                        connectTR[0][waypoints[in][jn]]=true;
                                    }
                                    if(connection((int)i,(int)j,in,jn,3)){
                                        graph[1][waypoints[in][jn]][waypoints[i][j]]=true;
                                        graph[1][waypoints[i][j]][waypoints[in][jn]]=true;
                                        connectTR[1][waypoints[i][j]]=true;
                                        connectTR[1][waypoints[in][jn]]=true;
                                    }
                                }
                            }
                        }
                    }
//                    int ii=max((int)i-1,0);
//                    int jj=max((int)j-1,0);

//                    if( getMapValue(2,i,j) && getMapValue(2,ii,j) && ii!=(int)i && waypoints[ii][j]>=0){
//                        graph[0][waypoints[ii][j]][waypoints[i][j]]=true;
//                        graph[0][waypoints[i][j]][waypoints[ii][j]]=true;
//                    }
//                    if( getMapValue(2,i,j) && getMapValue(2,i,jj) && jj!=(int)j && waypoints[i][jj]>=0){
//                        graph[0][waypoints[i][jj]][waypoints[i][j]]=true;
//                        graph[0][waypoints[i][j]][waypoints[i][jj]]=true;
//                    }

//                    if( getMapValue(3,i,j) && getMapValue(3,ii,j) && ii!=(int)i && waypoints[ii][j]>=0){
//                        graph[1][waypoints[ii][j]][waypoints[i][j]]=true;
//                        graph[1][waypoints[i][j]][waypoints[ii][j]]=true;
//                    }
//                    if( getMapValue(3,i,j) && getMapValue(3,i,jj) && jj!=(int)j && waypoints[i][jj]>=0){
//                        graph[1][waypoints[i][jj]][waypoints[i][j]]=true;
//                        graph[1][waypoints[i][j]][waypoints[i][jj]]=true;
//                    }

                    for(int m=max((int)i-r0s-1,0);m<=min((int)i+r0s+1,(int)waypoints.size()-1);m++){
                        for(int n=max((int)j-r0s-1,0);n<=min((int)j+r0s+1,(int)waypoints[i].size()-1);n++){
                            if( ((m-(int)i)*(m-(int)i)+(n-(int)j)*(n-(int)j))<=(r0s*r0s) ){
                                //if( getMapValue(2,i,j) && getMapValue(0,m,n) ){
                                if(connection_ray(i,j,m,n,2,0)){
                                    if(waypoints[m][n]>=0){
                                        visible[0][waypoints[i][j]][waypoints[m][n]]=true;
                                        visibleT[0][waypoints[m][n]]=true;
                                    }
                                }
                                if(connection_ray(i,j,m,n,6,0)){
                                    if(waypoints[m][n]>=0){
                                        visibleTR[0][waypoints[m][n]]=true;
                                    }
                                }
                            }
                        }
                    }

                    for(int m=max((int)i-r1s-1,0);m<=min((int)i+r1s+1,(int)waypoints.size()-1);m++){
                        for(int n=max((int)j-r1s-1,0);n<=min((int)j+r1s+1,(int)waypoints[i].size()-1);n++){
                            if( ((m-(int)i)*(m-(int)i)+(n-(int)j)*(n-(int)j))<=(r1s*r1s) ){
                                //if( getMapValue(3,i,j) && getMapValue(1,m,n) ){
                                if(connection_ray(i,j,m,n,3,1)){
                                    if(waypoints[m][n]>=0){
                                        visible[1][waypoints[i][j]][waypoints[m][n]]=true;
                                        visibleT[1][waypoints[m][n]]=true;
                                    }
                                }
                                if(connection_ray(i,j,m,n,7,1)){
                                    if(waypoints[m][n]>=0){
                                        visibleTR[1][waypoints[m][n]]=true;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        for(unsigned int i=0;i<waypoints.size();i++){
            for(unsigned int j=0;j<waypoints[i].size();j++){
                if(waypoints[i][j]>=0){
                    if(!visibleT[0][waypoints[i][j]] || (!visibleTR[0][waypoints[i][j]] && !connectTR[0][waypoints[i][j]])){
                        int imin=(int)round(max((float)i-(1.5*((float)jump)),0.0));
                        int jmin=(int)round(max((float)j-(1.5*((float)jump)),0.0));
                        int imax=(int)round(min((float)i+(1.5*((float)jump)),(double)waypoints[i].size()-1));
                        int jmax=(int)round(min((float)j+(1.5*((float)jump)),(double)waypoints[i].size()-1));
                        for(int in=imin;in<=imax;in++){
                            for(int jn=jmin;jn<=jmax;jn++){
                                if(waypoints[in][jn]>=0){
                                    if( max(abs(in-(int)i),abs(jn-(int)j))<=jump && (in!=(int)i || jn!=(int)j) ){
                                        if(connection_ray(in,jn,(int)i,(int)j,2,0)){
                                            visible[0][waypoints[in][jn]][waypoints[i][j]]=true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    if(!visibleT[1][waypoints[i][j]] || (!visibleTR[1][waypoints[i][j]] && !connectTR[1][waypoints[i][j]])){
                        int imin=(int)round(max((float)i-(1.5*((float)jump)),0.0));
                        int jmin=(int)round(max((float)j-(1.5*((float)jump)),0.0));
                        int imax=(int)round(min((float)i+(1.5*((float)jump)),(double)waypoints[i].size()-1));
                        int jmax=(int)round(min((float)j+(1.5*((float)jump)),(double)waypoints[i].size()-1));
                        for(int in=imin;in<=imax;in++){
                            for(int jn=jmin;jn<=jmax;jn++){
                                if(waypoints[in][jn]>=0){
                                    if( max(abs(in-(int)i),abs(jn-(int)j))<=jump && (in!=(int)i || jn!=(int)j) ){
                                        if(connection_ray(in,jn,(int)i,(int)j,3,1)){
                                            visible[1][waypoints[in][jn]][waypoints[i][j]]=true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        vector<int> goals(0);

        for(unsigned int i=0;i<waypoints.size();i++){
            for(unsigned int j=0;j<waypoints[i].size();j++){
                if( getMapValue(4,i,j) || getMapValue(5,i,j) ){
                    if(waypoints[i][j]>=0){
                        // only feasible for both robots in maze 3;
                        //if( (names[waypoints[i][j]].y>=21 && names[waypoints[i][j]].y<=33
                        //      && names[waypoints[i][j]].x>=30 && names[waypoints[i][j]].x<=52) ||
                        //    (names[waypoints[i][j]].y>=46 && names[waypoints[i][j]].y<=52
                        //      && names[waypoints[i][j]].x>=30 && names[waypoints[i][j]].x<=52) ||
                        //    (names[waypoints[i][j]].y>=21 && names[waypoints[i][j]].y<=52
                        //      && names[waypoints[i][j]].x>=49 && names[waypoints[i][j]].x<=52) )
                        //{
                            goals.push_back(waypoints[i][j]);
                        //}
                    }
                }
            }
        }

        vector<long int> initials(2,-1);
        //initials[0]=waypoints[pos_r0.x][pos_r0.y];
        //initials[1]=waypoints[pos_r1.x][pos_r1.y];
        vector<cv::Point> hlx=bf_hlx(jump+1);
        for(unsigned int h=0;h<hlx.size();h++){
            int px=pos_r0.x+hlx[h].x;
            int py=pos_r0.y+hlx[h].y;
            if(px>=0 && py>=0 && px<(int)waypoints.size() && py<(int)waypoints[0].size()){
                if(waypoints[px][py]>=0){
                    initials[0]=waypoints[px][py];
                    break;
                }
            }
        }
        for(unsigned int h=0;h<hlx.size();h++){
            int px=pos_r1.x+hlx[h].x;
            int py=pos_r1.y+hlx[h].y;
            if(px>=0 && py>=0 && px<(int)waypoints.size() && py<(int)waypoints[0].size()){
                if(waypoints[px][py]>=0){
                    initials[1]=waypoints[px][py];
                    break;
                }
            }
        }

        //cout<<pos_r0.x<<" "<<pos_r0.y<<" "<<pos_r1.x<<" "<<pos_r1.y<<" "<<endl;

        stringstream strstream("");
        write_preamble(strstream, 2,count, names);
        write_graph(strstream, graph, names);
        write_visible(strstream, visible, names);
        write_initRobots(strstream, 2, initials, names);
        write_goals(strstream, goals, names);//only feasible
        //write_goals(strstream, count, names);//total coverage, even if not feasible

        ofstream myfile;
        string pddlFolder = ros::package::getPath("map_transform").append("/pddl/problem.pddl");
        myfile.open (pddlFolder);
        if (myfile.is_open()){
            myfile << strstream.str();
        }
        else{
            ROS_INFO("Failed to open file.");
        }
        myfile.close();

        return true;
    }
    return false;
}

void PddlGen::publish(void){
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

        pub_mar1.publish(points);
        points.markers.clear();

        point.type=visualization_msgs::Marker::LINE_STRIP;
        point.color.b = 1.0f;
        point.color.r = 0.0f;
        point.color.g = 1.0f;
        point.scale.x = 0.03;
        point.scale.y = 0.02;
        point.scale.z = 0.001;
        geometry_msgs::Point p;
        p.x=0.0f; p.y=0.0f; p.z=0.0f;
        point.pose.position=p;

        for(unsigned int i=0;i<graph[0].size();i++){
            for(unsigned int j=0;j<graph[0][i].size();j++){
                if( graph[0][i][j] ){
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

        pub_mar2.publish(points);
        points.markers.clear();

        point.color.r = 1.0f;
        point.color.b = 0.0f;
        point.color.g = 1.0f;

        for(unsigned int i=0;i<visible[0].size();i++){
            for(unsigned int j=0;j<visible[0][i].size();j++){
                if( visible[0][i][j] ){
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

        pub_mar3.publish(points);
        points.markers.clear();

        point.color.b = 1.0f;
        point.color.r = 0.0f;
        point.color.g = 0.0f;

        for(unsigned int i=0;i<graph[1].size();i++){
            for(unsigned int j=0;j<graph[1][i].size();j++){
                if( graph[1][i][j] ){
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

        pub_mar4.publish(points);
        points.markers.clear();

        point.color.r = 1.0f;
        point.color.b = 0.0f;
        point.color.g = 0.0f;

        for(unsigned int i=0;i<visible[1].size();i++){
            for(unsigned int j=0;j<visible[1][i].size();j++){
                if( visible[1][i][j] ){
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

        pub_mar5.publish(points);
        points.markers.clear();
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

        pub_mar1.publish(points);
        pub_mar2.publish(points);
        pub_mar3.publish(points);
        pub_mar4.publish(points);
        pub_mar5.publish(points);
    }
}

bool suc=false;

// Replacement SIGINT handler
void HandlerStop(int)
{
    if(!suc)
        ROS_INFO("Failed generation of pddl.");
    ros::shutdown();
    exit(-1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner", ros::init_options::NoSigintHandler);
  signal(SIGINT, HandlerStop);

  ros::NodeHandle nh("~");

  PddlGen pddl(nh);

  ros::Rate loop_rate(10);

  while (ros::ok()){
    ros::spinOnce();
    if(!suc){
        if(pddl.plan()){
            suc=true;
            ROS_INFO("Successful generation of pddl.");
            //pddl.publish();
            //break;
        }
    }
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
