#include "ros/ros.h"
#include <vector>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_listener.h"
#include "std_srvs/Empty.h"
#include "Astar.hpp"
#include <map_transform/VisCom.h>
#include <map_transform/VisNode.h>
#include <map_transform/Regions.h>
#include <map_transform/PAstar.h>

#define V_MAP 0
#define E_MAP 2
#define O_MAP 4

using namespace std;

class Multirobotplannersensing{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub1, pub2;
    ros::Publisher pub_markers;
    ros::Subscriber sub1, sub2, sub3, sub4, sub5;
    ros::Subscriber graph_subscriber1, graph_subscriber2;
    ros::ServiceServer service;
    tf::TransformListener pos_listener;
    ros::ServiceClient regionsEditorService;
    vector<ros::ServiceClient> PAstarService;
    vector<vector<vector<bool> > >  msg_rcv;
    vector<bool> map_rcv;
    int width,height;
    float res;
    bool pl;
    vector<geometry_msgs::Point> regions;
    vector<PointI> goals;
    vector<vector<bool> > goals_map;
    std::vector<std::vector<float> > vis_;
    std::vector<std::vector<map_transform::VisNode> > crit_points;
    std::vector<bool> graph_rcv;
    string regionsEditorServiceName;
    string PAstarServiceSubName;
    vector<vector<map_transform::PAstarResponse> > bf_responses;
    void graphCallback(const map_transform::VisCom::ConstPtr& graph, int index);
    void graphCallback1(const map_transform::VisCom::ConstPtr& graph);
    void graphCallback2(const map_transform::VisCom::ConstPtr& graph);
    void rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg, int index);
    void rcv_map1(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map2(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map3(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map4(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map0(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    bool ask_plan(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
    bool getMapValue(int n, int i, int j);
    bool allAvailable();
    PointI convertW2I(geometry_msgs::Point p);
    geometry_msgs::Point convertI2W(PointI p);
    void regions2goals(void);
public:
    Multirobotplannersensing(ros::NodeHandle nh): nh_(nh){
        pub1 = nh_.advertise<nav_msgs::Path>("path0", 1,true);
        pub2 = nh_.advertise<nav_msgs::Path>("path1", 1,true);
        pub_markers = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1,true);
        sub1 = nh_.subscribe("robot_0/v_map", 1, &Multirobotplannersensing::rcv_map1, this);
        sub2 = nh_.subscribe("robot_1/v_map", 1, &Multirobotplannersensing::rcv_map2, this);
        sub3 = nh_.subscribe("robot_0/e_map", 1, &Multirobotplannersensing::rcv_map3, this);
        sub4 = nh_.subscribe("robot_1/e_map", 1, &Multirobotplannersensing::rcv_map4, this);
        sub5 = nh_.subscribe("map", 1, &Multirobotplannersensing::rcv_map0, this);
        graph_subscriber1 = nh.subscribe("robot_0/graph", 10 , &Multirobotplannersensing::graphCallback1, this);
        graph_subscriber2 = nh.subscribe("robot_1/graph", 10 , &Multirobotplannersensing::graphCallback2, this);
        service = nh_.advertiseService("plan", &Multirobotplannersensing::ask_plan, this);
        map_rcv.assign(5,false);
        msg_rcv.resize(5);
        vis_.resize(2);
        crit_points.resize(2);
        graph_rcv.assign(2, false);
        bf_responses.resize(2);
        pl=false;
        goals.clear();
        regions.clear();
        nh_.param("goalRegionsService", regionsEditorServiceName, string("/regionsEditor/getRegions"));
        nh_.param("PAstarService", PAstarServiceSubName, string("/planner/plan_point"));
        regionsEditorService = nh_.serviceClient<map_transform::Regions>(regionsEditorServiceName);
        regionsEditorService.waitForExistence();
        PAstarService.resize(2);
        PAstarService[0]=nh_.serviceClient<map_transform::PAstar>("/robot_0"+PAstarServiceSubName);
        PAstarService[0].waitForExistence();
        PAstarService[1]=nh_.serviceClient<map_transform::PAstar>("/robot_1"+PAstarServiceSubName);
        PAstarService[1].waitForExistence();
    }
    ~Multirobotplannersensing(){
    }
    void plan(void);
    void publish(void);
};

void Multirobotplannersensing::graphCallback(const map_transform::VisCom::ConstPtr& graph, int index){
    vis_[index]=graph->vis;
    crit_points[index]=graph->crit_points;
    graph_rcv[index]=true;
}

void Multirobotplannersensing::graphCallback1(const map_transform::VisCom::ConstPtr& graph){
    graphCallback(graph, 0);
}

void Multirobotplannersensing::graphCallback2(const map_transform::VisCom::ConstPtr& graph){
    graphCallback(graph, 1);
}

void Multirobotplannersensing::rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg, int index){
    msg_rcv[index].assign(msg->info.width, vector<bool>(msg->info.height,false));
    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
            if(*mapDataIterC == 0){
                msg_rcv[index][j][i]=true;
            }
            else{
                msg_rcv[index][j][i]=false;
            }
            mapDataIterC++;
        }
    }
    map_rcv[index]=true;
    if(index==O_MAP){
        res=msg->info.resolution;
        width=msg->info.width;
        height=msg->info.height;
        goals_map.assign(msg->info.width, vector<bool>(msg->info.height,false));
    }
}

void Multirobotplannersensing::rcv_map0(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg,O_MAP);
}

void Multirobotplannersensing::rcv_map1(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg,V_MAP);
}

void Multirobotplannersensing::rcv_map2(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg,V_MAP+1);
}

void Multirobotplannersensing::rcv_map3(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg,E_MAP);
}

void Multirobotplannersensing::rcv_map4(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    rcv_map(msg,E_MAP+1);
}

bool Multirobotplannersensing::getMapValue(int n, int i, int j){
    return msg_rcv[n][i][j];
}

bool Multirobotplannersensing::ask_plan(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res){
    if(!allAvailable())
        return false;
    map_transform::Regions srv;
    if(!regionsEditorService.call(srv)){
        pl=false;
        return false;
    }
    goals_map.assign(width, vector<bool>(height,false));
    goals.clear();
    regions.clear();
    for(unsigned int i=0; i<srv.response.regions.poses.size(); i++){
        regions.push_back(srv.response.regions.poses[i].position);
    }
    pl=true;
    return true;
}

PointI Multirobotplannersensing::convertW2I(geometry_msgs::Point p){
    return PointI(round((p.x-0.5*res)/res),round((p.y-0.5*res)/res));
}

geometry_msgs::Point Multirobotplannersensing::convertI2W(PointI p){
    geometry_msgs::Point pf;
    pf.x=(p.i+0.5)*res;
    pf.y=(p.j+0.5)*res;
    return pf;
}


//double Multirobotplannersensing::distance2Path(vector<PointI> path, PointI goal, int r)
//{
//    vector<double> heuristic(0);
//    vector<int> pos(0);

//    for(unsigned int p=0;p<path.size();p++)
//    {
//        if (!msg_rcv[r][path[p].i][path[p].j])
//            continue;

//        double heur=(double) (path[p].i-goal.i)*(path[p].i-goal.i)+(path[p].j-goal.j)*(path[p].j-goal.j);

//        vector<double>::iterator heurIt = heuristic.begin();
//        vector<int>::iterator posIt = pos.begin();

//        while(heurIt!=heuristic.end())
//        {
//            if(heur<*heurIt)
//                break;
//            heurIt++;
//            posIt++;
//        }

//        heuristic.insert(heurIt,heur);
//        pos.insert(posIt,p);
//    }

//    double min_real=0;
//    Apath pathT;

//    for(unsigned int h=0;h<(heuristic.size());h++)
//    {
//        pathT=Astar(path[pos[h]], goal,r);
//        if(h==0)
//            min_real=pathT.cost;
//        else
//        {
//            if(pathT.cost<min_real)
//                min_real=pathT.cost;

//            if(h<(heuristic.size()-1))
//            {
//                if( (min_real*min_real)<heuristic[h+1])
//                    return min_real;
//                else
//                    continue;
//            }
//        }
//    }

//    return pathT.cost;
//}

//int Multirobotplannersensing::minDistanceGoal2Path(vector<PointI> path, vector<PointI> goal, int r)
//{
//    vector<double> heuristic(0);
//    vector<int> pos(0);
//    vector<int> gp(0);

//    for(unsigned int g=0;g<goal.size();g++)
//    {
//        for( unsigned int p=0;p<path.size();p++)
//        {
//            if (!msg_rcv[r][path[p].i][path[p].j])
//                continue;

//            double heur=(double) (path[p].i-goal[g].i)*(path[p].i-goal[g].i)+(path[p].j-goal[g].j)*(path[p].j-goal[g].j);

//            vector<double>::iterator heurIt = heuristic.begin();
//            vector<int>::iterator posIt = pos.begin();
//            vector<int>::iterator gpIt = gp.begin();

//            while(heurIt!=heuristic.end())
//            {
//                if(heur<*heurIt)
//                    break;
//                heurIt++;
//                posIt++;
//                gpIt++;
//            }

//            heuristic.insert(heurIt,heur);
//            pos.insert(posIt,p);
//            gp.insert(gpIt,g);
//        }
//    }

//    double min_real=0;
//    int g_real=0;
//    Apath pathT;

//    for(unsigned int h=0;h<(heuristic.size());h++)
//    {
//        pathT=Astar(path[pos[h]], goal[gp[h]],r);
//        if(h==0)
//        {
//            min_real=pathT.cost;
//            g_real=gp[h];
//        }
//        else
//        {
//            if(pathT.cost<min_real)
//            {
//                min_real=pathT.cost;
//                g_real=gp[h];
//            }

//            if(h<(heuristic.size()-1))
//            {
//                if( (min_real*min_real)<heuristic[h+1])
//                    return g_real;
//                else
//                    continue;
//            }
//        }
//    }

//    return g_real;
//}

bool Multirobotplannersensing::allAvailable(void){
    return map_rcv[0] && map_rcv[0] && map_rcv[0] && map_rcv[0] && map_rcv[0];
}

void Multirobotplannersensing::regions2goals(void){
    goals_map.assign(width, vector<bool>(height,false));
    goals.clear();
    for(unsigned int i=0; i<regions.size(); i++){
        geometry_msgs::Point pt=regions[i];
        PointI pi=convertW2I(pt);
        int rad=(int)round(pt.z/res);
        for(int row=pi.i-rad; row<=pi.i+rad; row++){
            for(int col=pi.j-rad; col<=pi.j+rad; col++){
                int ii=row-pi.i;
                int jj=col-pi.j;
                if(width>0 && height>0 && ( (ii*ii+jj*jj)<=(rad*rad) ) )
                    if(row>=0 && col>=0 && row<(int)msg_rcv[O_MAP].size() && col<(int)msg_rcv[O_MAP][0].size())
                        if(getMapValue(O_MAP, row, col))
                            if(!goals_map[row][col]){
                                goals.push_back(PointI(row, col));
                                goals_map[row][col]=true;
                            }
            }
        }
    }
}

void Multirobotplannersensing::plan(void){
    if(!allAvailable() || !pl ){
        return;
    }
    nav_msgs::Path path_0,path_1;

    vector<vector<PointI> > pr(2,vector<PointI>(0));
    vector< vector<int> > pg(2,vector<int>(0)), gr(4,vector<int>(0));
    vector<PointI> gi;

    path_0.poses.clear();
    path_0.header.frame_id = "/map";
    path_0.header.stamp =  ros::Time::now();
    path_1=path_0;

    geometry_msgs::PoseStamped p_temp;
    p_temp.header.frame_id   = "/map";
    p_temp.header.stamp =  ros::Time::now();
    p_temp.pose.orientation.w=1;
    tf::StampedTransform transform;
    try{
        pos_listener.lookupTransform("/map", "/robot_0/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_INFO("%s",ex.what());
      return ;
    }
    PointI pi_temp( (int) round((transform.getOrigin().x()-0.5*res)/res), (int) round((transform.getOrigin().y()-0.5*res)/res) );
    p_temp.pose.position=convertI2W(pi_temp);
    path_0.poses.push_back(p_temp);
    pr[0].push_back(pi_temp);
    gi.push_back(pr[0].back());
    try{
        pos_listener.lookupTransform("/map", "/robot_1/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_INFO("%s",ex.what());
      return ;
    }
    pi_temp=PointI( (int) round((transform.getOrigin().x()-0.5*res)/res), (int) round((transform.getOrigin().y()-0.5*res)/res) );
    p_temp.pose.position=convertI2W(pi_temp);
    path_1.poses.push_back(p_temp);
    pr[1].push_back(pi_temp);
    gi.push_back(pr[1].back());

    regions2goals();
    for(unsigned int i=0;i<goals.size();i++){
        gi.push_back(goals[i]);
        pi_temp=goals[i];
        if( msg_rcv[V_MAP][pi_temp.i][pi_temp.j] && !msg_rcv[V_MAP+1][pi_temp.i][pi_temp.j] )
            gr[0].push_back(2+i);
        else if ( !msg_rcv[V_MAP][pi_temp.i][pi_temp.j] && msg_rcv[V_MAP+1][pi_temp.i][pi_temp.j] )
            gr[1].push_back(2+i);
        else if ( msg_rcv[V_MAP][pi_temp.i][pi_temp.j] && msg_rcv[V_MAP+1][pi_temp.i][pi_temp.j] )
            gr[2].push_back(2+i);
        else
            gr[3].push_back(2+i);
    }
    pg[0].push_back(0);
    pg[1].push_back(1);

    ROS_INFO("%lu %lu %lu %lu", gr[0].size(), gr[1].size(), gr[2].size(), gr[3].size());

    //vector<bool> gt(g.size(),false);
    //int g0s=gr[0].size(), g1s=gr[1].size(),gbs=gr[2].size();
    //vector<vector<vector<Apath> > > mat(g.size(),vector<vector<Apath> >(g.size(),vector<Apath>(2,Apath())));
    //Apath path;

    /////////////////////////// METHOD 1 ///////////////////////////////////////

    ros::Time t0=ros::Time::now();
    cout<<"----> Method 1 (BF):"<<endl;

    bf_responses[0].assign(goals.size(), map_transform::PAstarResponse());
    bf_responses[1].assign(goals.size(), map_transform::PAstarResponse());

    map_transform::PAstar srv;
    vector<int> count(4, 0);
    for(int i=0; i<2; i++){
        for(unsigned int g=0; g<goals.size(); g++){
            ROS_INFO("%d %d !!!!! %d %d", i, i, g, g);
            srv.request.goal=convertI2W(goals[g]);
            if(PAstarService[i].call(srv)){
                bf_responses[i][g]=srv.response;
                count[i]++;
            }
            else{
                bf_responses[i][g].cost=-10;
                count[i+2]++;
            }
        }
    }

    ros::Duration diff = ros::Time::now() - t0;
    ROS_INFO("Time for brute force of PAstar: %f", diff.toSec());
    ROS_INFO("%d %d %d %d", count[0], count[1], count[2], count[3]);

    pl=false;
}

void Multirobotplannersensing::publish(void){
    if(goals.size()>0){
        visualization_msgs::MarkerArray points;
        visualization_msgs::Marker point;
        point.header.frame_id = "/map";
        point.header.stamp =  ros::Time::now();
        point.ns = "points_and_lines";
        point.action = visualization_msgs::Marker::ADD;
        point.pose.orientation.w = 1.0;
        point.type = visualization_msgs::Marker::SPHERE;
        point.scale.x = res;
        point.scale.y = res;
        point.scale.z = 0.05;
        point.color.g = 1.0;
        point.color.a = 1.0;
        point.lifetime = ros::Duration(2);
        for (uint32_t i = 0; i < goals.size(); ++i){
          point.pose.position=convertI2W(goals[i]);
          point.id = i;
          points.markers.push_back(point);
        }
        pub_markers.publish(points);
    }
}

int main(int argc, char **argv){ 
  ros::init(argc, argv, "multi_sensing_planning");
  ros::NodeHandle nh("~");
  Multirobotplannersensing planner(nh);
  ros::Rate loop_rate(10);
  while (ros::ok()){
    ros::spinOnce();
    planner.plan();
    planner.publish();
    loop_rate.sleep();
  }
  return 0;
}