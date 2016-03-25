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
#include "vector_utils.hpp"

using namespace std;

class PddlGen{
private:
    ros::NodeHandle nh_;

    ros::Subscriber sub1, sub2, sub3, sub4, sub5, sub6;

    vector<vector<vector<bool> > >  msg_rcv;
    vector<int> count;
    vector<bool> map_rcv;
    float res;
    int width,height;

    int r0s, r1s;

    tf::TransformListener pos_listener;

    void rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg, int index);
    void rcv_map1(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map2(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map3(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map4(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map5(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map6(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    bool getMapValue(int n, int i, int j);
    cv::Point2i convertW2I(geometry_msgs::Point p);
    geometry_msgs::Point convertI2W(cv::Point2i p);
    bool connection(int i, int j, int in, int jn, int r);
public:
    PddlGen(ros::NodeHandle nh): nh_(nh)
    {
        sub1 = nh_.subscribe("/robot_0/c_map", 1, &PddlGen::rcv_map1, this);
        sub2 = nh_.subscribe("/robot_1/c_map", 1, &PddlGen::rcv_map2, this);
        sub3 = nh_.subscribe("/robot_0/e_map", 1, &PddlGen::rcv_map3, this);
        sub4 = nh_.subscribe("/robot_1/e_map", 1, &PddlGen::rcv_map4, this);
        sub5 = nh_.subscribe("/robot_0/v_map", 1, &PddlGen::rcv_map5, this);
        sub6 = nh_.subscribe("/robot_1/v_map", 1, &PddlGen::rcv_map6, this);
        count.assign(6,0);
        map_rcv.assign(6,false);
        msg_rcv.resize(6);

        nh_.param("/robot_0/visibility/infl", r0s, 5);
        nh_.param("/robot_1/visibility/infl", r1s, 5);

        //cout<<r0s<<" "<<r1s<<endl;
    }

    bool plan(void);
};

void PddlGen::rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg, int index)
{
    msg_rcv[index].assign(msg->info.width, vector<bool>(msg->info.height,false));

    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
            if(*mapDataIterC == 0)
            {
                msg_rcv[index][j][i]=true;
            }
            else
            {
                msg_rcv[index][j][i]=false;
            }

            mapDataIterC++;
        }
    }

    map_rcv[index]=true;

    count[index]=count[index]+1;

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
    pf.x=p.x*res;
    pf.y=p.y*res;
    return pf;
}

string convertG2S(cv::Point2i pt){
    string str;
    str.append(to_string(pt.x)).append(";").append(to_string(pt.y));
    return str;
}

void write_preamble(stringstream & str, int nr, int nw, vector<cv::Point2i> names)
{
    str<<"(define (problem robotprob1) (:domain robot-building)"<<endl<<endl;
    str<<"(objects\n\t";
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

bool PddlGen::connection(int i, int j, int in, int jn, int r){
    int imin=min(i,in);
    int imax=max(i,in);

    int jmin=min(j,jn);
    int jmax=max(j,jn);

    for(int ii=imin;ii<=imax;ii++){
        for(int jj=jmin;jj<=jmax;jj++){
            if(!getMapValue(r,ii,jj))
                return false;
        }
    }
    return true;
}

bool PddlGen::plan(void){
    if(map_rcv[0] && map_rcv[1] && map_rcv[2] && map_rcv[3] && map_rcv[4] && map_rcv[5]){
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

        int jump=min(r0s,r1s)/2;
        //int jump=6;

        for(unsigned int i=0;i<waypoints.size();i=i+jump){
            unsigned int in=i/jump;
            for(unsigned int j=0;j<waypoints[i].size();j=j+jump){
                unsigned int jn=j/jump;
                if( getMapValue(0,i,j) || getMapValue(1,i,j) ){
                    waypoints[i][j]=count;
                    names.push_back(cv::Point(in,jn));
                    count++;
                }
            }
        }

        vector<vector<vector<bool> > > graph(2, vector<vector<bool> >(count, vector<bool>(count,false)));

        vector<vector<vector<bool> > > visible(2, vector<vector<bool> >(count, vector<bool>(count,false)));

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
                                if((in==(int)i || jn==(int)j) && (in!=(int)i || jn!=(int)j) ){
                                    if(connection((int)i,(int)j,in,jn,2)){
                                        graph[0][waypoints[in][jn]][waypoints[i][j]]=true;
                                        graph[0][waypoints[i][j]][waypoints[in][jn]]=true;
                                    }
                                    if(connection((int)i,(int)j,in,jn,3)){
                                        graph[1][waypoints[in][jn]][waypoints[i][j]]=true;
                                        graph[1][waypoints[i][j]][waypoints[in][jn]]=true;
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
                                if( getMapValue(2,i,j) && getMapValue(0,m,n) ){
                                    if(waypoints[i][j]>=0 && waypoints[m][n]>=0)
                                        visible[0][waypoints[i][j]][waypoints[m][n]]=true;
                                }
                            }
                        }
                    }

                    for(int m=max((int)i-r1s-1,0);m<=min((int)i+r1s+1,(int)waypoints.size()-1);m++){
                        for(int n=max((int)j-r1s-1,0);n<=min((int)j+r1s+1,(int)waypoints[i].size()-1);n++){
                            if( ((m-(int)i)*(m-(int)i)+(n-(int)j)*(n-(int)j))<=(r1s*r1s) ){
                                if( getMapValue(3,i,j) && getMapValue(1,m,n) ){
                                    if(waypoints[i][j]>=0 && waypoints[m][n]>=0)
                                        visible[1][waypoints[i][j]][waypoints[m][n]]=true;
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
                    if(waypoints[i][j]>=0)
                        goals.push_back(waypoints[i][j]);
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
        //write_goals(strstream, goals);//only feasible
        write_goals(strstream, count, names);//total coverage, even if not feasible

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

// Replacement SIGINT handler
void HandlerStop(int)
{
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

  bool suc=false;

  while (ros::ok())
  {
    ros::spinOnce();
    if(pddl.plan())
    {
        suc=true;
        break;
    }
    loop_rate.sleep();
  }

  if(!suc)
      ROS_INFO("Failed generation of pddl!");
  else
      ROS_INFO("Successful generation of pddl.");

  ros::shutdown();

  return 0;
}
