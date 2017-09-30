#include "ros/ros.h"
#include <vector>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_listener.h"
#include <map_transform/VisCom.h>
#include <map_transform/VisNode.h>
#include <map_transform/PAstarSrv.h>
#include <fstream>
#include <queue>
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "PAstar.hpp"
#include "points_conversions.hpp"
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include "color.hpp"

using namespace std;

//#define LAMBDA 0.007
#define LAMBDA 0.04

class Planner{
private:
    int infl;
    int defl;
    int vvv;
    unsigned int clock;
    nav_msgs::Path path_0, path_1;
    cv::Mat or_map, nav_map, vis_map, r_map;
    vector<cv::Mat> deb1, deb2, deb3, deb4, deb5, deb6, deb7;
    cv::Point target, p0_ini;
    ros::NodeHandle nh_;
    ros::Publisher pub1, pub2;
    ros::Publisher pub_markers,pub_rob_markers;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Subscriber sub3, sub4;
    ros::Subscriber graph_subscriber;
    ros::Subscriber sub_goals;
    ros::ServiceServer service, service3;
    ros::ServiceServer service2;
    tf::TransformListener pos_listener;
    std::string tf_pref;
    std::vector<float> vis_;
    std::vector<map_transform::VisNode> crit_points;
    vector<vector<vector<bool> > >  msg_rcv;
    vector<int> count;
    vector<bool> map_rcv;
    float res;
    int width,height;
    bool graph_rcv;
    bool pl;
    PAstar pastar;
    vector<geometry_msgs::Point> goals;
    void rcv_map1(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map2(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map3(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map4(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_goal(const geometry_msgs::PoseStamped::ConstPtr& msg);
    bool getMapValue(int n, int i, int j);
    float convertCostI2W(float cost);
    void graphCallback(const map_transform::VisCom::ConstPtr& graph);
    bool isGoal(PointI p0, PointI p1);
    void clearG();
    bool clear(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
    bool ask_plan(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
    bool request_single_plan(map_transform::PAstarSrv::Request  &req, map_transform::PAstarSrv::Response &res);
    bool planFromRequest(geometry_msgs::Point goal, float & cost, geometry_msgs::Point & perc_pt);
public:
    Planner(ros::NodeHandle nh, bool server_mode=false): nh_(nh){
        pub1 = nh_.advertise<nav_msgs::Path>("path0", 1,true);
        pub_markers = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1,true);
        pub_rob_markers = nh_.advertise<visualization_msgs::MarkerArray>("visualization_robot_marker_array", 1,true);
        sub1 = nh_.subscribe("v_map", 1, &Planner::rcv_map1, this);
        sub2 = nh_.subscribe("e_map", 1, &Planner::rcv_map2, this);
        sub3 = nh_.subscribe("map", 1, &Planner::rcv_map3, this);
        sub4 = nh_.subscribe("r_map", 1, &Planner::rcv_map4, this);
        service3 = nh_.advertiseService("plan_point", &Planner::request_single_plan, this);
        service2 = nh_.advertiseService("clear", &Planner::clear, this);
        if(!server_mode){
            graph_subscriber = nh.subscribe("graph", 10 , &Planner::graphCallback, this);
            pub2 = nh_.advertise<nav_msgs::Path>("path1", 1,true);
            sub_goals = nh_.subscribe("/move_base_simple/goal", 1, &Planner::rcv_goal, this);
            service = nh_.advertiseService("plan", &Planner::ask_plan, this);
        }
        nh_.param("tf_prefix", tf_pref, std::string("/robot_0"));
        nh_.param("infl", infl, 5);
        nh_.param("defl", defl, infl);
        count.assign(4,0);
        map_rcv.assign(4,false);
        msg_rcv.resize(4);
        graph_rcv=false;
        pl=false;
        goals.clear();
        pastar=PAstar(infl, defl);
        vvv=-1;
        clock=0;
    }
    ~Planner(){

    }
    void plan(void);
    void publish(void);
};

void Planner::graphCallback(const map_transform::VisCom::ConstPtr& graph){
    vis_=graph->vis;
    crit_points=graph->crit_points;
    graph_rcv=true;
}

void Planner::rcv_map1(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    msg_rcv[0].assign(msg->info.width, vector<bool>(msg->info.height,false));
    vis_map = cv::Mat(msg->info.width, msg->info.height, CV_8UC1);
    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
            if(*mapDataIterC == 0){
                msg_rcv[0][j][i]=true;
                vis_map.at<uchar>(j,i) = 255;
            }
            else{
                msg_rcv[0][j][i]=false;
                vis_map.at<uchar>(j,i) = 0;
            }
            mapDataIterC++;
        }
    }
    map_rcv[0]=true;
    count[0]=count[0]+1;
    res=msg->info.resolution;
    width=msg->info.width;
    height=msg->info.height;
}

void Planner::rcv_map2(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    msg_rcv[1].assign(msg->info.width, vector<bool>(msg->info.height,false));
    nav_map = cv::Mat(msg->info.width, msg->info.height, CV_8UC1);
    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
            if(*mapDataIterC == 0){
                msg_rcv[1][j][i]=true;
                nav_map.at<uchar>(j,i) = 255;
            }
            else{
                msg_rcv[1][j][i]=false;
                nav_map.at<uchar>(j,i) = 0;
            }
            mapDataIterC++;
        }
    }
    map_rcv[1]=true;
    count[1]=count[1]+1;

    pastar.updateNavMap(msg_rcv[1]);
}

void Planner::rcv_map3(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    msg_rcv[2].assign(msg->info.width, vector<bool>(msg->info.height,false));
    or_map = cv::Mat(msg->info.width, msg->info.height, CV_8UC1);
    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
            if(*mapDataIterC == 0){
                msg_rcv[2][j][i]=true;
                or_map.at<uchar>(j,i) = 255;
            }
            else{
                msg_rcv[2][j][i]=false;
                or_map.at<uchar>(j,i) = 0;
            }
            mapDataIterC++;
        }
    }
    map_rcv[2]=true;
    count[2]=count[2]+1;

    pastar.updateOrMap(or_map);
}

void Planner::rcv_map4(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    msg_rcv[3].assign(msg->info.width, vector<bool>(msg->info.height,false));
    r_map = cv::Mat(msg->info.width, msg->info.height, CV_8UC1);
    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
            if(*mapDataIterC == 0){
                msg_rcv[3][j][i]=true;
                r_map.at<uchar>(j,i) = 255;
            }
            else{
                msg_rcv[3][j][i]=false;
                r_map.at<uchar>(j,i) = 0;
            }
            mapDataIterC++;
        }
    }
    map_rcv[3]=true;
    count[3]=count[3]+1;

    pastar.updateOrMap(or_map);
}

bool Planner::getMapValue(int n, int i, int j){
    return msg_rcv[n][i][j];
}

void Planner::clearG(){
    pl=false;
    goals.clear();
    path_0.poses.clear();
    path_0.header.frame_id = "/map";
    path_0.header.stamp =  ros::Time::now();
    path_0.poses.clear();
    path_1=path_0;
    pub1.publish(path_0);
    pub2.publish(path_1);
}

bool Planner::clear(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res){
    clearG();
    return true;
}

void Planner::rcv_goal(const geometry_msgs::PoseStamped::ConstPtr& msg){
    //clearG();
    goals.push_back(msg->pose.position);
    pl=true;
}

bool Planner::ask_plan(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res){
    pl=true;
    return true;
}

bool Planner::request_single_plan(map_transform::PAstarSrv::Request  &req, map_transform::PAstarSrv::Response &res){
    geometry_msgs::Point perc_pt;
    float cost;
    clearG();
    goals.push_back(req.goal);
    planFromRequest(req.goal, cost, perc_pt);
    res.cost=cost;
    res.perc_pt=perc_pt;
    return true;
}

float Planner::convertCostI2W(float cost){
    return cost*res;
}

//int index_file=0;
//ofstream myfile[14];

void Planner::plan(void){
    if(map_rcv[0] && map_rcv[1] && map_rcv[2] && map_rcv[3] && graph_rcv && pl && (goals.size()>0) ){
        path_0.poses.clear();
        path_0.header.frame_id = "/map";
        path_0.header.stamp =  ros::Time::now();
        geometry_msgs::PoseStamped pw;
        pw.header.frame_id   = "/map";
        pw.header.stamp =  ros::Time::now();
        pw.pose.orientation.w=1;
        tf::StampedTransform transform;
        try{
            pos_listener.lookupTransform("/map", tf_pref+"/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_INFO("%s",ex.what());
          return ;
        }
        geometry_msgs::Point p;
        p.x=transform.getOrigin().x();
        p.y=transform.getOrigin().y();
        PointI pi;
        PApath path;
        pl=false;

        ROS_INFO("Planning!!");

        //path=pastar.run(pi, convertW2I(goals[0], res), LAMBDA, true, -5,true);

//        for(unsigned int tt=0;tt<2;tt++){
//            for(unsigned ll=0;ll<7;ll++){
//                k1=1;
//                if(ll==0)
//                    k2=0.008;
//                else
//                    k2*=5;
//                if(tt==0)
//                    index_file=ll;
//                else
//                    index_file=7+ll;
//                if(tt==0)
//                    {quad=false;myfile[index_file]<<"Linear\n";}
//                else
//                    {quad=true;myfile[index_file]<<"Quadratic\n";}
//                myfile[index_file]<<"Lambda="<<k2/k1<<"\n";
//                for(int n=1;n<9;n++){
//                    string xsx;
//                    if(n==1)
//                        {p.x=10; p.y=10; xsx="sim_1";}
//                    else if(n==2)
//                        {p.x=8.3; p.y=7.5; xsx="sim_2"; //continue;}
//                    else if(n==3)
//                        {p.x=11.7; p.y=6.2; xsx="sim_3";//continue;}
//                    else if(n==4)
//                        {p.x=15; p.y=10.7; xsx="sim_4";//continue;}
//                    else if(n==5)
//                        {p.x=11.6; p.y=14; xsx="sim_5";//continue;}
//                    else if(n==6)
//                        {p.x=17.3; p.y=4.5; xsx="sim_6";//continue;}
//                    else if(n==7)
//                        {p.x=18.3; p.y=16.3; xsx="sim_7";//continue;}
//                    else if(n==8)
//                        {p.x=18.3; p.y=11.7; xsx="sim_8";//continue;}
//                    myfile[index_file]<<xsx<<"\n";

        pi=convertWtf2I(p, res);

        int save_rate=50;
        //for(unsigned int ii=max((int)(goals.size()-1), 0); ii<goals.size();ii++){
        //for(unsigned int ii=0; ii<goals.size();ii++){
        for(unsigned int ii=0; ii<1;ii++){
            ROS_INFO("Goal %u",ii);
            ros::Time t01=ros::Time::now();
            ros::Duration diff;
            //for(unsigned int i=ii; i<ii+1;i++){
            //for(unsigned int i=ii; i<goals.size();i++){
            for(unsigned int i=(goals.size()-1); i<goals.size();i++){
                PointI g=convertW2I(goals[i], res);
                if(g.i<0 || g.i>=(int)msg_rcv[0].size()){
                    //clearG();
                    path.cost=-3;
                    continue;
                }
                if(g.j<0 || g.j>=(int)msg_rcv[0][g.i].size()){
                    //clearG();
                    path.cost=-3;
                    continue;
                }
                if(!msg_rcv[2][g.i][g.j]){
                    //clearG();
                    path.cost=-3;
                    continue;
                }
                path=pastar.run(pi, g, LAMBDA, true, -3, true,NULL,NULL,false,false, NULL, NULL, save_rate);
                cout<<"ExpTUS: "<<path.exp_unfiltered<<"; Exp: "<<path.exp_nodes<<"; Exp_r: "<<path.exp_nodes_r<<"; Goal_tested: "<<path.tested_goal<<endl;
                target=cv::Point(g.i,g.j);
                p0_ini=cv::Point(pi.i,pi.j);
            }
            deb1=pastar.getExpansions();
            //deb1.assign(1,cv::Mat::zeros(or_map.rows,or_map.cols,CV_8UC1));
            diff = ros::Time::now() - t01;
            //if(path.cost!=-2)
            //    myfile[index_file]<<diff<<"; "<<path.cost<<"; ";
            ROS_INFO("Time BFS: %f; Cost: %f",diff.toSec(),path.cost);
            //path=pastar.run(pi, convertW2I(goals[0], res), LAMBDA, true, -5);
            t01=ros::Time::now();
            //for(unsigned int i=ii; i<ii+1;i++){
            //for(unsigned int i=ii; i<goals.size();i++){
            for(unsigned int i=(goals.size()-1); i<goals.size();i++){
                PointI g=convertW2I(goals[i], res);
                if(g.i<0 || g.i>=(int)msg_rcv[0].size()){
                    //clearG();
                    path.cost=-3;
                    continue;
                }
                if(g.j<0 || g.j>=(int)msg_rcv[0][g.i].size()){
                    //clearG();
                    path.cost=-3;
                    continue;
                }
                if(!msg_rcv[2][g.i][g.j]){
                    //clearG();
                    path.cost=-3;
                    continue;
                }
                path=pastar.run(pi, g, LAMBDA, true,-3, false,NULL,NULL,false,false, NULL, NULL, save_rate);
                cout<<"ExpTUS: "<<path.exp_unfiltered<<"; Exp: "<<path.exp_nodes<<"; Exp_r: "<<path.exp_nodes_r<<"; Goal_tested: "<<path.tested_goal<<endl;
            }
            deb2=pastar.getExpansions();
            diff = ros::Time::now() - t01;
            //if(path.cost>=0)
            //    myfile[index_file]<<diff<<"; "<<path.cost<<"; ";
            ROS_INFO("Time PA: %f; Cost: %f",diff.toSec(),path.cost);
            path_0.poses.clear();
            path_1=path_0;
            if(path.cost>0){
                if(path.points.size()!=0)
                    for(unsigned int p_i=0;p_i<path.points.size();p_i++){
                        pw.pose.position=convertI2W(path.points[p_i], res);
                        path_0.poses.push_back(pw);
                    }
                else{
                    pw.pose.position=p;
                    path_0.poses.push_back(pw);
                }
            }
            pub1.publish(path_0);

            //path=pastar.run(pi, convertW2I(goals[0], res),LAMBDA, true, -5);
            bool run=false;
            if(vis_.size()==(msg_rcv[0].size()*msg_rcv[0][0].size())){
                t01=ros::Time::now();
                //for(unsigned int i=ii; i<ii+1;i++){
                //for(unsigned int i=ii; i<goals.size();i++){
                for(unsigned int i=(goals.size()-1); i<goals.size();i++){
                    run=false;
                    PointI g=convertW2I(goals[i], res);
                    if(g.i<0 || g.i>=(int)msg_rcv[0].size()){
                        //clearG();
                        path.cost=-3;
                        continue;
                    }
                    if(g.j<0 || g.j>=(int)msg_rcv[0][g.i].size()){
                        //clearG();
                        path.cost=-3;
                        continue;
                    }
                    if(!msg_rcv[0][g.i][g.j]){
                        //clearG();
                        path.cost=-3;
                        continue;
                    }
                    path=pastar.run(pi, g, LAMBDA, true, vis_[g.i*msg_rcv[0][0].size()+g.j], false,NULL,NULL,false,false, NULL, NULL, save_rate);
                    cout<<"ExpTUS: "<<path.exp_unfiltered<<"; Exp: "<<path.exp_nodes<<"; Exp_r: "<<path.exp_nodes_r<<"; Goal_tested: "<<path.tested_goal<<endl;
                    run=true;
                }
                deb3=pastar.getExpansions();
                //deb3.assign(1,cv::Mat::zeros(or_map.rows,or_map.cols,CV_8UC1));
                diff = ros::Time::now() - t01;
                ROS_INFO("Time PA-RDVM (1): %f; Cost: %f",diff.toSec(),path.cost);
            }
            if(!run){
                deb3.assign(1,cv::Mat::zeros(or_map.rows,or_map.cols,CV_8UC1));
            }

            //path=pastar.run(pi, convertW2I(goals[0], res),LAMBDA, true, -5);
            run=false;
            if(vis_.size()==(msg_rcv[0].size()*msg_rcv[0][0].size())){
                t01=ros::Time::now();
                //for(unsigned int i=ii; i<ii+1;i++){
                //for(unsigned int i=ii; i<goals.size();i++){
                for(unsigned int i=(goals.size()-1); i<goals.size();i++){
                    run=false;
                    PointI g=convertW2I(goals[i], res);
                    if(g.i<0 || g.i>=(int)msg_rcv[0].size()){
                        //clearG();
                        path.cost=-3;
                        continue;
                    }
                    if(g.j<0 || g.j>=(int)msg_rcv[0][g.i].size()){
                        //clearG();
                        path.cost=-3;
                        continue;
                    }
                    if(!msg_rcv[0][g.i][g.j]){
                        //clearG();
                        path.cost=-3;
                        continue;
                    }
                    path=pastar.run(pi, g, LAMBDA, true, vis_[g.i*msg_rcv[0][0].size()+g.j], false, NULL, NULL, true,false, NULL, NULL, save_rate);
                    cout<<"ExpTUS: "<<path.exp_unfiltered<<"; Exp: "<<path.exp_nodes<<"; Exp_r: "<<path.exp_nodes_r<<"; Goal_tested: "<<path.tested_goal<<endl;
                    run=true;
                }
                deb4=pastar.getExpansions();
                //deb4.assign(1,cv::Mat::zeros(or_map.rows,or_map.cols,CV_8UC1));
                diff = ros::Time::now() - t01;
                ROS_INFO("Time PA-RDVM (1+CD): %f; Cost: %f",diff.toSec(),path.cost);
            }
            if(!run){
                deb4.assign(1,cv::Mat::zeros(or_map.rows,or_map.cols,CV_8UC1));
            }

            //path=pastar.run(pi, convertW2I(goals[0], res),LAMBDA, true, -5);
             run=false;
             if(vis_.size()==(msg_rcv[0].size()*msg_rcv[0][0].size())){
                t01=ros::Time::now();
                //for(unsigned int i=ii; i<ii+1;i++){
                //for(unsigned int i=ii; i<goals.size();i++){
                for(unsigned int i=(goals.size()-1); i<goals.size();i++){
                    run=false;
                    PointI g=convertW2I(goals[i], res);
                    if(g.i<0 || g.i>=(int)msg_rcv[0].size()){
                        //clearG();
                        path.cost=-3;
                        continue;
                    }
                    if(g.j<0 || g.j>=(int)msg_rcv[0][g.i].size()){
                        //clearG();
                        path.cost=-3;
                        continue;
                    }
                    if(!msg_rcv[0][g.i][g.j]){
                        //clearG();
                        path.cost=-3;
                        continue;
                    }
                    path=pastar.run(pi, g, LAMBDA, true, vis_[g.i*msg_rcv[0][0].size()+g.j], false, &crit_points[g.i*msg_rcv[0][0].size()+g.j], NULL, true,false, NULL, NULL, save_rate );
                    cout<<"ExpTUS: "<<path.exp_unfiltered<<"; Exp: "<<path.exp_nodes<<"; Exp_r: "<<path.exp_nodes_r<<"; Goal_tested: "<<path.tested_goal<<endl;
                    run=true;
                }
                deb5=pastar.getExpansions();
                //deb5.assign(1,cv::Mat::zeros(or_map.rows,or_map.cols,CV_8UC1));
                diff = ros::Time::now() - t01;
                ROS_INFO("Time PA-RDVM (1+CD+2): %f; Cost: %f",diff.toSec(),path.cost);
            }
            if(!run){
                deb5.assign(1,cv::Mat::zeros(or_map.rows,or_map.cols,CV_8UC1));
            }

            //path=pastar.run(pi, convertW2I(goals[0], res),LAMBDA, true, -5);
            run=false;
            if(vis_.size()==(msg_rcv[0].size()*msg_rcv[0][0].size())){
                t01=ros::Time::now();
                //for(unsigned int i=ii; i<ii+1;i++){
                //for(unsigned int i=ii; i<goals.size();i++){
                for(unsigned int i=(goals.size()-1); i<goals.size();i++){
                    run=false;
                    PointI g=convertW2I(goals[i], res);
                    if(g.i<0 || g.i>=(int)msg_rcv[0].size()){
                        //clearG();
                        path.cost=-3;
                        continue;
                    }
                    if(g.j<0 || g.j>=(int)msg_rcv[0][g.i].size()){
                        //clearG();
                        path.cost=-3;
                        continue;
                    }
                    if(!msg_rcv[0][g.i][g.j]){
                        //clearG();
                        path.cost=-3;
                        continue;
                    }
                    path=pastar.run(pi, g, LAMBDA, true, vis_[g.i*msg_rcv[0][0].size()+g.j], false, &crit_points[g.i*msg_rcv[0][0].size()+g.j], NULL, true, true, NULL, NULL, save_rate);
                    //cout<<"ExpTUS: "<<path.exp_unfiltered<<"; Exp: "<<path.exp_nodes<<"; Exp_r: "<<path.exp_nodes_r<<"; Goal_tested: "<<path.tested_goal<<endl;
                    run=true;
                }
                deb6=pastar.getExpansions();
                //deb6.assign(1,cv::Mat::zeros(or_map.rows,or_map.cols,CV_8UC1));
                diff = ros::Time::now() - t01;
                //ROS_INFO("Time PA-RDVM (1-OS+2-CS): %f; Cost: %f",diff.toSec(),path.cost);
            }
            if(!run){
                deb6.assign(1,cv::Mat::zeros(or_map.rows,or_map.cols,CV_8UC1));
            }

            //path=pastar.run(pi, convertW2I(goals[0], res),LAMBDA, true, -5);
            run=false;
            if(vis_.size()==(msg_rcv[0].size()*msg_rcv[0][0].size())){
                t01=ros::Time::now();
                //for(unsigned int i=ii; i<ii+1;i++){
                //for(unsigned int i=ii; i<goals.size();i++){
                for(unsigned int i=(goals.size()-1); i<goals.size();i++){
                    run=false;
                    PointI g=convertW2I(goals[i], res);
                    if(g.i<0 || g.i>=(int)msg_rcv[0].size()){
                        //clearG();
                        path.cost=-3;
                        continue;
                    }
                    if(g.j<0 || g.j>=(int)msg_rcv[0][g.i].size()){
                        //clearG();
                        path.cost=-3;
                        continue;
                    }
                    if(!msg_rcv[0][g.i][g.j]){
                        //clearG();
                        path.cost=-3;
                        continue;
                    }
                    vector<float> crits_dists;
                    vector<float> crits_angles;
                    vector<float> crits_anglesDelta;
                    if(vis_[g.i*msg_rcv[0][0].size()+g.j]>=0){
                        for(unsigned int cc=0; cc<crit_points[g.i*msg_rcv[0][0].size()+g.j].points.size(); cc++){
                            float xG=g.i-crit_points[g.i*msg_rcv[0][0].size()+g.j].points[cc].position.x;
                            float yG=g.j-crit_points[g.i*msg_rcv[0][0].size()+g.j].points[cc].position.y;
                            float distCG=sqrt(xG*xG+yG*yG);
                            crits_dists.push_back(distCG);
                            crits_angles.push_back(atan2(yG,xG));
                            //crits_anglesDelta.push_back(atan2(infl,crits_dists.back()));
                            crits_anglesDelta.push_back(atan2(infl,sqrt(distCG*distCG-infl*infl)));
                        }
                    }
                    path=pastar.run(pi, g, LAMBDA, true, vis_[g.i*msg_rcv[0][0].size()+g.j], false, &crit_points[g.i*msg_rcv[0][0].size()+g.j], &crits_dists, true, true, &crits_angles, &crits_anglesDelta, save_rate);
                    cout<<"ExpTUS: "<<path.exp_unfiltered<<"; Exp: "<<path.exp_nodes<<"; Exp_r: "<<path.exp_nodes_r<<"; Goal_tested: "<<path.tested_goal<<endl;
                    run=true;
                }
                deb7=pastar.getExpansions();
                diff = ros::Time::now() - t01;
                ROS_INFO("Time PA-RDVM (1+CD+2+CP): %f; Cost: %f",diff.toSec(),path.cost);
                //if(path.cost>=0)
                //    myfile[index_file]<<diff<<"; "<<path.cost<<"; "<<"\n";
                if(path.cost>0){
                    if(path.points.size()!=0)
                        for(unsigned int p_i=0;p_i<path.points.size();p_i++){
                            pw.pose.position=convertI2W(path.points[p_i], res);
                            path_1.poses.push_back(pw);
                        }
                    else{
                        pw.pose.position=p;
                        path_1.poses.push_back(pw);
                    }
                }
                pub2.publish(path_1);
            }
            if(!run){
                deb7.assign(1,cv::Mat::zeros(or_map.rows,or_map.cols,CV_8UC1));
            }
        }
        clock=0;
        vvv=0;
    }
    unsigned char c_b[3]={0,0,0};
    unsigned char c_w[3]={255,255,255};
    unsigned char c_n[3]={100,100,100};
    unsigned char c_v[3]={150,150,150};
    unsigned char c_o[3]={0,100,250};
    unsigned char c_c[3]={100,0,250};
    unsigned char c_cf[3]={180,0,250};
    unsigned char c_g[3]={255,90,80};
    unsigned char c_p[3]={255,0,0};
    unsigned char c_t[3]={0,255,0};
    unsigned char c_i[3]={0,255,255};
    if(vvv>=0 && deb7.size()>0 && deb7[0].rows>0){
        //cv::imshow("BFS",color_print_expansion(or_map,nav_map,nav_map,deb1[min(vvv,(int)deb1.size()-1)],target,p0_ini,c_b,c_w,c_n,c_v,c_o,c_c,c_cf,c_g,c_p,c_t,c_i));
        //cv::waitKey(3);
        cv::imshow("PA",color_print_expansion(or_map,nav_map,nav_map,deb2[min(vvv,(int)deb2.size()-1)],target,p0_ini,c_b,c_w,c_n,c_v,c_o,c_c,c_cf,c_g,c_p,c_t,c_i));
        cv::waitKey(3);
        cv::imshow("PA 1",color_print_expansion(or_map,r_map,vis_map,deb3[min(vvv,(int)deb3.size()-1)],target,p0_ini,c_b,c_w,c_n,c_v,c_o,c_c,c_cf,c_g,c_p,c_t,c_i));
        cv::waitKey(3);
        cv::imshow("PA 1+CD",color_print_expansion(or_map,r_map,vis_map,deb4[min(vvv,(int)deb4.size()-1)],target,p0_ini,c_b,c_w,c_n,c_v,c_o,c_c,c_cf,c_g,c_p,c_t,c_i));
        cv::waitKey(3);
        cv::imshow("PA 1+CD+2",color_print_expansion(or_map,r_map,vis_map,deb5[min(vvv,(int)deb5.size()-1)],target,p0_ini,c_b,c_w,c_n,c_v,c_o,c_c,c_cf,c_g,c_p,c_t,c_i));
        cv::waitKey(3);
        //cv::imshow("PA 1+CD+2+CP",color_print_expansion(or_map,r_map,vis_map,deb6[min(vvv,(int)deb6.size()-1)],target,p0_ini,c_b,c_w,c_n,c_v,c_o,c_c,c_cf,c_g,c_p,c_t,c_i));
        //cv::waitKey(3);
        cv::imshow("PA 1+CD+2+CP",color_print_expansion(or_map,r_map,vis_map,deb7[min(vvv,(int)deb7.size()-1)],target,p0_ini,c_b,c_w,c_n,c_v,c_o,c_c,c_cf,c_g,c_p,c_t,c_i));
        cv::waitKey(3);
        clock++;
        if(clock==1){
            clock=0;
            if(vvv<(int)deb2.size())
                vvv++;
            //cout<<"loop "<<vvv<<endl;
        }
        //cout<<"Test "<<clock<<endl;
    }
//    if(deb7.size()>0 && deb7[0].rows>0){
//        cv::imshow("BFS",color_print_expansion(or_map,nav_map,nav_map,deb1.back(),target,c_b,c_w,c_n,c_v,c_o,c_c,c_cf,c_g,c_p,c_t));
//        cv::waitKey(3);
//        cv::imshow("PA",color_print_expansion(or_map,nav_map,nav_map,deb2.back(),target,c_b,c_w,c_n,c_v,c_o,c_c,c_cf,c_g,c_p,c_t));
//        cv::waitKey(3);
//        cv::imshow("PA 1",color_print_expansion(or_map,r_map,vis_map,deb3.back(),target,c_b,c_w,c_n,c_v,c_o,c_c,c_cf,c_g,c_p,c_t));
//        cv::waitKey(3);
//        cv::imshow("PA 1-OS",color_print_expansion(or_map,r_map,vis_map,deb4.back(),target,c_b,c_w,c_n,c_v,c_o,c_c,c_cf,c_g,c_p,c_t));
//        cv::waitKey(3);
//        cv::imshow("PA 1-OS 2",color_print_expansion(or_map,r_map,vis_map,deb5.back(),target,c_b,c_w,c_n,c_v,c_o,c_c,c_cf,c_g,c_p,c_t));
//        cv::waitKey(3);
//        cv::imshow("PA 1-OS 2-CS-P",color_print_expansion(or_map,r_map,vis_map,deb7.back(),target,c_b,c_w,c_n,c_v,c_o,c_c,c_cf,c_g,c_p,c_t));
//        cv::waitKey(3);
//    }
}

void Planner::publish(void){
    visualization_msgs::MarkerArray points;
    visualization_msgs::Marker point;
    if(goals.size()>0){
        point.header.frame_id = "/map";
        point.header.stamp =  ros::Time::now();
        point.ns = "points_and_lines";
        point.action = visualization_msgs::Marker::ADD;
        point.pose.orientation.w = 1.0;
        point.type = visualization_msgs::Marker::SPHERE;
        point.scale.x = 0.2;
        point.scale.y = 0.2;
        point.scale.z = 0.02;
        point.color.g = 1.0f;
        point.color.a = 1.0;
        point.lifetime = ros::Duration(0.15);
        for (uint32_t i = 0; i < goals.size(); ++i){
          point.pose.position=goals[i];
          point.id = i;
          points.markers.push_back(point);
        }
        if(path_0.poses.size()>0){
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
            p=path_0.poses[path_0.poses.size()-1].pose.position;
            point.points.clear();
            point.points.push_back(p);
            point.points.push_back(goals[goals.size()-1]);
            point.id = goals.size();
            points.markers.push_back(point);
        }
        if(path_1.poses.size()>0){
            point.type=visualization_msgs::Marker::LINE_STRIP;
            point.color.r = 1.0f;
            point.color.b = 0.0f;
            point.color.g = 1.0f;
            point.scale.x = 0.03;
            point.scale.y = 0.02;
            point.scale.z = 0.001;
            geometry_msgs::Point p;
            p.x=0.0f; p.y=0.0f; p.z=0.0f;
            point.pose.position=p;
            p=path_1.poses[path_1.poses.size()-1].pose.position;
            point.points.clear();
            point.points.push_back(p);
            point.points.push_back(goals[goals.size()-1]);
            point.id = goals.size()+1;
            points.markers.push_back(point);
        }
    }
    else{
        visualization_msgs::MarkerArray points;
        visualization_msgs::Marker point;
        point.header.frame_id = "/map";
        point.header.stamp =  ros::Time::now();
        point.ns = "points_and_lines";
        point.action = 3;
        points.markers.push_back(point);
    }
    pub_markers.publish(points);

    visualization_msgs::MarkerArray robots;
    visualization_msgs::Marker robot;
    robot.header.frame_id = "/robot_0/base_link";
    robot.header.stamp =  ros::Time::now();
    robot.ns = "robot0";
    robot.id=0;
    robot.action = visualization_msgs::Marker::ADD;
    robot.pose.orientation.w = 1.0;
    robot.type = visualization_msgs::Marker::SPHERE;
    robot.scale.x = 2.0*(float)infl*res;
    robot.scale.y = 2.0*(float)infl*res;
    robot.scale.z = 0.01;
    robot.color.r = 1.0;
    robot.color.a = 1.0;
    robot.lifetime = ros::Duration(1);
    robots.markers.push_back(robot);
    pub_rob_markers.publish(robots);
}

bool Planner::planFromRequest(geometry_msgs::Point goal, float & cost, geometry_msgs::Point & perc_pt){
    if(map_rcv[0] && map_rcv[1] && map_rcv[2]){
        path_0.poses.clear();
        path_0.header.frame_id = "/map";
        path_0.header.stamp =  ros::Time::now();
        geometry_msgs::PoseStamped pw;
        pw.header.frame_id   = "/map";
        pw.header.stamp =  ros::Time::now();
        pw.pose.orientation.w=1;
        PointI gi=convertW2I(goal, res);
        if(gi.i<0 || gi.i>=(int)msg_rcv[0].size()){
            cost=-3;
            return false;
        }
        if(gi.j<0 || gi.j>=(int)msg_rcv[0][gi.i].size()){
            cost=-3;
            return false;
        }
        if(!msg_rcv[2][gi.i][gi.j]){
            cost=-4;
            return false;
        }
        tf::StampedTransform transform;
        try{
            pos_listener.lookupTransform("/map", tf_pref+"/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_INFO("%s",ex.what());
          cost=-5;
          return false;
        }
        geometry_msgs::Point pt;
        pt.x=transform.getOrigin().x();
        pt.y=transform.getOrigin().y();
        PointI pi=convertWtf2I(pt, res);
        //pastar.run(pi, gi, LAMBDA, true, -5);
        PApath path=pastar.run(pi, gi, LAMBDA, true);
        if(path.cost>=0){
            path_0.poses.clear();
            if(path.points.size()!=0){
                for(unsigned int p_i=0;p_i<path.points.size();p_i++){
                    pw.pose.position=convertI2W(path.points[p_i], res);
                    path_0.poses.push_back(pw);
                }
                perc_pt=pw.pose.position;
            }
            else{
                pw.pose.position=pt;
                path_0.poses.push_back(pw);
                perc_pt=pw.pose.position;
            }
            cost=convertCostI2W(path.cost);
            pub1.publish(path_0);
        }
        else{
            cost=-1;
            return false;
        }
        return true;
    }
    else{
        cost=-2;
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh("~");
//    myfile[0].open ("/home/viki/rosbuild_ws/map_transform/resultsL1.txt", ios::out);
//    myfile[1].open ("/home/viki/rosbuild_ws/map_transform/resultsL2.txt", ios::out);
//    myfile[2].open ("/home/viki/rosbuild_ws/map_transform/resultsL3.txt", ios::out);
//    myfile[3].open ("/home/viki/rosbuild_ws/map_transform/resultsL4.txt", ios::out);
//    myfile[4].open ("/home/viki/rosbuild_ws/map_transform/resultsL5.txt", ios::out);
//    myfile[5].open ("/home/viki/rosbuild_ws/map_transform/resultsL6.txt", ios::out);
//    myfile[6].open ("/home/viki/rosbuild_ws/map_transform/resultsL7.txt", ios::out);
//    myfile[7].open ("/home/viki/rosbuild_ws/map_transform/resultsQ1.txt", ios::out);
//    myfile[8].open ("/home/viki/rosbuild_ws/map_transform/resultsQ2.txt", ios::out);
//    myfile[9].open ("/home/viki/rosbuild_ws/map_transform/resultsQ3.txt", ios::out);
//    myfile[10].open ("/home/viki/rosbuild_ws/map_transform/resultsQ4.txt", ios::out);
//    myfile[11].open ("/home/viki/rosbuild_ws/map_transform/resultsQ5.txt", ios::out);
//    myfile[12].open ("/home/viki/rosbuild_ws/map_transform/resultsQ6.txt", ios::out);
//    myfile[13].open ("/home/viki/rosbuild_ws/map_transform/resultsQ7.txt", ios::out);

    bool server_mode=false;

    if(argc>=2){
        if(string(argv[1])=="--server")
            server_mode=true;
    }

    Planner planner(nh, server_mode);
    ros::Rate loop_rate(10);
    while (ros::ok()){
        ros::spinOnce();
        planner.plan();
        planner.publish();
        loop_rate.sleep();
    }
//    for(int i=0;i<14;i++)
//    myfile[i].close();
  return 0;
}
