#include "ros/ros.h"
#include <vector>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_listener.h"
#include "std_srvs/Empty.h"
#include <map_transform/VisCom.h>
#include <map_transform/VisNode.h>
#include <map_transform/Regions.h>
#include <map_transform/PAstarSrv.h>
#include "PAstar.hpp"
#include "points_conversions.hpp"
#include "clustering.hpp"
#include "vector_utils.hpp"

#include "combinatorics.hpp"

#define V_MAP 0
#define E_MAP 2
#define O_MAP 4

using namespace std;

class PAresult{
public:
    float cost, costM, costP;
    PointI perc_pt, p0;
    PAresult(){
        cost=-1;
        costM=-1;
        costP=-1;
        perc_pt=PointI();
        p0=PointI();
    }
};

class Multirobotplannersensing{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub1, pub2, pub3, pub4, pub5;
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
    cv::Mat or_map;
    vector<geometry_msgs::Point> regions;
    vector<PointI> goals;
    vector<vector<int> > goalsRegionsIds;
    vector<vector<int> > goals_map;
    vector<vector<int> > goalsInRegions;
    std::vector<std::vector<float> > vis_;
    std::vector<std::vector<map_transform::VisNode> > crit_points;
    std::vector<bool> graph_rcv;
    string regionsEditorServiceName;
    string PAstarServiceSubName;
    vector<int> infl, defl;
    vector<vector<vector<PAresult> > > bf_responses;
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
    void regions2goals(void);
    float evaluate(vector<vector<vector<float> > > costM, vector<bool> feasible_goals, vector<vector<int> > paths, vector<float> goal_costs, vector<bool> goal_visited, float th=-1);
    float evaluate(vector<vector<vector<float> > > costM, vector<vector<vector<float> > > costP, vector<bool> feasible_goals, vector<vector<int> > paths, float th=-1);
    float costSensing(float K, float dist);
public:
    Multirobotplannersensing(ros::NodeHandle nh): nh_(nh){
        pub1 = nh_.advertise<nav_msgs::Path>("path0", 1,true);
        pub2 = nh_.advertise<nav_msgs::Path>("path1", 1,true);
        pub3 = nh_.advertise<nav_msgs::Path>("path2", 1,true);
        pub4 = nh_.advertise<nav_msgs::Path>("path3", 1,true);
        pub5 = nh_.advertise<nav_msgs::Path>("path4", 1,true);
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
        PAstarService[0]=nh_.serviceClient<map_transform::PAstarSrv>("/robot_0"+PAstarServiceSubName);
        PAstarService[0].waitForExistence();
        PAstarService[1]=nh_.serviceClient<map_transform::PAstarSrv>("/robot_1"+PAstarServiceSubName);
        PAstarService[1].waitForExistence();
        infl.resize(2);
        defl.resize(2);
        nh_.param("robot_0/infl", infl[0], 8);
        nh_.param("robot_1/infl", infl[1], 8);
        nh_.param("robot_0/defl", defl[0], 80);
        nh_.param("robot_1/defl", defl[1], 80);
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
    if(index==O_MAP)
        or_map = cv::Mat(msg->info.width, msg->info.height, CV_8UC1);
    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
            if(*mapDataIterC == 0){
                msg_rcv[index][j][i]=true;
                if(index==O_MAP)
                    or_map.at<uchar>(j,i) = 255;
            }
            else{
                msg_rcv[index][j][i]=false;
                if(index==O_MAP)
                    or_map.at<uchar>(j,i) = 0;
            }
            mapDataIterC++;
        }
    }
    map_rcv[index]=true;
    if(index==O_MAP){
        res=msg->info.resolution;
        width=msg->info.width;
        height=msg->info.height;
        goals_map.assign(msg->info.width, vector<int>(msg->info.height,-1));
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
    goals_map.assign(width, vector<int>(height,-1));
    goals.clear();
    goalsRegionsIds.clear();
    regions.clear();
    for(unsigned int i=0; i<srv.response.regions.poses.size(); i++){
        regions.push_back(srv.response.regions.poses[i].position);
    }
    pl=true;
    return true;
}

bool Multirobotplannersensing::allAvailable(void){
    return map_rcv[0] && map_rcv[0] && map_rcv[0] && map_rcv[0] && map_rcv[0];
}

void Multirobotplannersensing::regions2goals(void){
    goals_map.assign(width, vector<int>(height,-1));
    goals.clear();
    goalsRegionsIds.clear();
    goalsInRegions.assign(regions.size(), vector<int>(0));
    for(unsigned int i=0; i<regions.size(); i++){
        geometry_msgs::Point pt=regions[i];
        PointI pi=convertW2I(pt, res);
        int rad=(int)round(pt.z/res);
        for(int row=pi.i-rad; row<=pi.i+rad; row++){
            for(int col=pi.j-rad; col<=pi.j+rad; col++){
                int ii=row-pi.i;
                int jj=col-pi.j;
                if(width>0 && height>0 && ( (ii*ii+jj*jj)<=(rad*rad) ) )
                    if(row>=0 && col>=0 && row<(int)msg_rcv[O_MAP].size() && col<(int)msg_rcv[O_MAP][0].size())
                        if(getMapValue(O_MAP, row, col)){
                            if(goals_map[row][col]<0){
                                goals.push_back(PointI(row, col));
                                goals_map[row][col]=goals.size()-1;
                                goalsRegionsIds.push_back(vector<int>(1,i));
                                goalsInRegions[i].push_back(goals.size()-1);
                            }
                            else{
                                goalsRegionsIds[goals_map[row][col]].push_back(i);
                                goalsInRegions[i].push_back(goals_map[row][col]);
                            }
                        }
            }
        }
    }
}

void Multirobotplannersensing::plan(void){
    if(!allAvailable() || !pl ){
        return;
    }
    nav_msgs::Path path_0,path_1, path_2, path_3, path_4;

    vector<vector<PointI> > pr(2,vector<PointI>(0));
    vector< vector<int> > pg(2,vector<int>(0)), gr(4,vector<int>(0));
    vector<PointI> gi;

    path_0.poses.clear();
    path_0.header.frame_id = "/map";
    path_0.header.stamp =  ros::Time::now();
    path_1=path_0;
    path_2=path_0;
    path_3=path_0;
    path_4=path_0;

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
    PointI pi_temp( (int) round((transform.getOrigin().x())/res), (int) round((transform.getOrigin().y())/res) );
    p_temp.pose.position=convertI2W(pi_temp, res);
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
    pi_temp=PointI( (int) round((transform.getOrigin().x())/res), (int) round((transform.getOrigin().y())/res) );
    p_temp.pose.position=convertI2W(pi_temp, res);
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

    cout<<"----> Method 1 (BF):"<<endl;
    ros::Time t0=ros::Time::now();

    bf_responses[0].assign(goals.size(), vector<PAresult>(0, PAresult()));
    bf_responses[1].assign(goals.size(), vector<PAresult>(0, PAresult()));

    vector<vector<int> > paths(2, vector<int>(1, 0));
    vector<vector<PointI> > clusterPts(2, vector<PointI>(0));

    vector<vector<float> > newMaxValue(2, vector<float>(goals.size(), -1));

    vector<vector<vector<float> > > costsMs;
    vector<vector<vector<float> > > costsPs;
    vector<bool> feasible_goals(goals.size(), false);
    vector<float> goals_costs;
    vector<bool> goals_visited;

    vector<int> count(4, 0);
    for(int i=0; i<2; i++){
    //for(int i=0; i<1; i++){
        cout<<"Robot "<<i<<endl;
        PAstar pastar(infl[i], defl[i]);
        pastar.updateNavMap(msg_rcv[E_MAP+i]);
        pastar.updateOrMap(or_map);
        vector<PointI> clusterpoints(0);
        vector<int> clusterpointsID(0);
        clusterpoints.push_back(pr[i].front());
        clusterpointsID.push_back(-1);
        vector<int> clusterVisitOrder(0);
        vector<PointI> cluster_centers(0);
        vector<vector<vector<bool> > > goalsInClusters(0);
        int n=0;
        while(!clusterpoints.empty()){
            PointI p0=clusterpoints.back();
            int curr_cluster=clusterpointsID.back();
            clusterpoints.pop_back();
            clusterpointsID.pop_back();
            if((curr_cluster+1)>=(int)clusterVisitOrder.size()){
                clusterVisitOrder.resize(cluster_centers.size()+1, -1);
            }
            clusterVisitOrder[curr_cluster+1]=n;
            cout<<"Robot "<<i<<"; From Point "<<n<<"; "<<p0.i<<" "<<p0.j<<"; order: "<<curr_cluster<<endl;
            n++;
            vector<cv::Point> points(0);
            vector<vector<vector<int> > > pointGoals(width, vector<vector<int> >(height,vector<int>(0)));
            for(unsigned int g=0; g<goals.size(); g++){
                PApath path=pastar.run(p0, goals[g], 0.04, true);
                PAresult paresult;
                paresult.p0=p0;
                if(path.cost>=0){
                    paresult.cost=path.cost;
                    paresult.costM=path.costM;
                    paresult.costP=path.costP;
                    if(path.points.size()!=0){
                        paresult.perc_pt=path.points.back();
                    }
                    else{
                        paresult.perc_pt=p0;
                    }
                    count[i]++;
                    points.push_back(cv::Point(paresult.perc_pt.i,paresult.perc_pt.j));
                    pointGoals[points.back().x][points.back().y].push_back(g);
                }
                else{
                    count[i+2]++;
                }
                bf_responses[i][g].push_back(paresult);
            }
            vector<vector<cv::Point> > clusters=cluster_points(points);
            for(unsigned int cc=0; cc<clusters.size(); cc++){
                float posi=0, posj=0;
                for(unsigned int pp=0; pp<clusters[cc].size();pp++){
                    posi+=clusters[cc][pp].x;
                    posj+=clusters[cc][pp].y;
                }
                posi/=clusters[cc].size();
                posj/=clusters[cc].size();
                FindMin<unsigned long int, PointI> med;
                for(unsigned int pp=0; pp<clusters[cc].size();pp++){
                    PointI pt(clusters[cc][pp].x,clusters[cc][pp].y);
                    med.iter(pt.diff2(PointI(posi,posj)),pt);
                }
                FindMin<unsigned long int, PointI> closest_cluster;
                for(unsigned int pp=0; pp<cluster_centers.size();pp++){
                    closest_cluster.iter(cluster_centers[pp].diff2(med.getP()),cluster_centers[pp]);
                }
                bool new_cluster=false;
                unsigned long int threshold=infl[i];
                if(closest_cluster.getVal()>(threshold*threshold)){
                    new_cluster=true;
                }
                else{
                    Apath path=Astar<float>(closest_cluster.getP(), med.getP(), msg_rcv[E_MAP+i], false, infl[i]);//1*closest_cluster.getVal());
                    if(path.cost<0){
                        new_cluster=true;
                    }
                }
                int active_cluster;
                if(new_cluster){
                    cluster_centers.push_back(med.getP());
                    clusterpoints.push_back(med.getP());
                    clusterpointsID.push_back(cluster_centers.size()-1);
                    goalsInClusters.push_back(vector<vector<bool> >(cluster_centers.size()+1, vector<bool>(goals.size(), false)));
                    active_cluster=goalsInClusters.size()-1;
                }
                else{
                    active_cluster=closest_cluster.getInd();
                }
                if((curr_cluster+1)>=(int)goalsInClusters[active_cluster].size())
                    goalsInClusters[active_cluster].resize(cluster_centers.size()+1,vector<bool>(goals.size(), false));
                for(unsigned int pp=0; pp<clusters[cc].size();pp++){
                    for(unsigned int gg=0; gg<pointGoals[clusters[cc][pp].x][clusters[cc][pp].y].size(); gg++){
                        goalsInClusters[active_cluster][curr_cluster+1][pointGoals[clusters[cc][pp].x][clusters[cc][pp].y][gg]]=true;
                    }
                }
            }
        }
        vector<float> maxValue(goals.size(), -1);
        float max_dist=-1;
        vector<vector<float> > costsM(cluster_centers.size()+1,vector<float>(cluster_centers.size()+1,-1));
        vector<vector<float> > costsP(cluster_centers.size(),vector<float>(goals.size(),-1));
        for(unsigned int pt=0; pt<cluster_centers.size();pt++){
            for(unsigned int other=0; other<=cluster_centers.size(); other++){
                float sum=0, number=0;
                if(other<goalsInClusters[pt].size() ){
                    for(unsigned int goal=0; goal<goalsInClusters[pt][other].size(); goal++){
                        if(goalsInClusters[pt][other][goal]){
                            if(bf_responses[i][goal][clusterVisitOrder[other]].cost>=0 ){
                                sum+=bf_responses[i][goal][clusterVisitOrder[other]].costM;
                                number++;
                                float new_cost_p=bf_responses[i][goal][clusterVisitOrder[other]].costP;
                                if( costsP[pt][goal]<0 && new_cost_p>=0 ){
                                    feasible_goals[goal]=true;
                                    newMaxValue[i][goal]=costSensing(0.04,(float)defl[i]);
                                    costsP[pt][goal]=new_cost_p;
                                    if( maxValue[goal]<costsP[pt][goal] ){
                                        maxValue[goal]=costsP[pt][goal];
                                    }
                                }
                                else if(new_cost_p>=0 && (costsP[pt][goal]>new_cost_p) ){
                                    costsP[pt][goal]=new_cost_p;
                                    if( maxValue[goal]<costsP[pt][goal] ){
                                        maxValue[goal]=costsP[pt][goal];
                                    }
                                }
                            }
                        }
                    }
                }
                number=0;
                if(number==0){
                    PointI endpt;
                    if(other==0)
                        endpt=pr[i].front();
                    else
                        endpt=cluster_centers[other-1];
                    Apath newpath=Astar<float>(cluster_centers[pt],endpt, msg_rcv[E_MAP+i]);
                    sum=newpath.cost;
                }
                else{
                    sum=sum/number;
                }
                float max_temp;
                if( costsM[pt+1][other]<0){
                    costsM[pt+1][other]=sum;
                    costsM[other][pt+1]=sum;
                    max_temp=sum;
                }
                else{
                    max_temp=max(costsM[pt+1][other], sum);
                    costsM[pt+1][other]=(costsM[pt+1][other]+sum)/2;
                    costsM[other][pt+1]=costsM[pt+1][other];
                }
                if(max_temp>max_dist){
                    max_dist=max_temp;
                }
            }
        }
        costsMs.push_back(costsM);
        costsPs.push_back(costsP);
        if(i>0)
            continue;
        vector<float> goal_costs=newMaxValue[i];//=maxValue;
        vector<bool> goal_visited(goals.size(), false);
        vector<PointI> points=cluster_centers;
        vector<bool> points_visited(points.size(), false);
        unsigned int num_visited=0;
        bool finished=false;
        while(!finished){
        //while(!points.empty()){
            FindMax<float, unsigned int> best_point;
            for(unsigned int pt=0; pt<points.size();pt++){
                if(!points_visited[pt]){
                    float gained_cost=0;
                    FindMin<float> min_pos;
                    for(unsigned int pos=0; pos<(paths[i].size()-1); pos++){
                        min_pos.iter(costsM[paths[i][pos]][pt+1]+costsM[pt+1][paths[i][pos+1]]-costsM[paths[i][pos]][paths[i][pos+1]]);
                        cout<<paths[i][pos]<<"; "<<pt+1<<"; "<<paths[i][pos+1]<<"; "<<costsM[paths[i][pos]][pt+1]<<"; "<<costsM[paths[i][pos+1]][pt+1]<<"; "<<costsM[paths[i][pos]][paths[i][pos+1]]<<"; "<<costsM[paths[i][pos]][pt+1]+costsM[pt+1][paths[i][pos+1]]-costsM[paths[i][pos]][paths[i][pos+1]]<<endl;
                    }
                    min_pos.iter(costsM[paths[i].back()][pt+1]);
                    cout<<costsM[paths[i].back()][pt+1]<<endl;
                    gained_cost+=-min_pos.getVal();
                    cout<<"Point "<<pt<<"; motion: "<<min_pos.getVal()<<"; ";
                    bool new_goal=false;
                    for(auto goal=0u; goal<goals.size(); goal++){
                        if(maxValue[goal]>=0 && costsP[pt][goal]>=0){
                            if( costsP[pt][goal]<= goal_costs[goal] ){
                                if(!new_goal && !goal_visited[goal]){
                                    new_goal=true;
                                    gained_cost+=max_dist;
                                    cout<<"Special; ";
                                }
                                for(auto region=0u; region<goalsRegionsIds[goal].size(); region++){
                                    gained_cost+=(goal_costs[goal]-costsP[pt][goal])/goalsInRegions[goalsRegionsIds[goal][region]].size();
                                    //cout<<(goal_costs[goal]-costsP[pt][goal])/goalsInRegions[goalsRegionsIds[goal][region]].size()<<";";
                                }
                            }
                        }
                    }
                    cout<<"\nFinal: "<<gained_cost<<endl;
                    //for(unsigned int other=0; other<points.size(); other++){
                    //}
                    best_point.iter(gained_cost, min_pos.getInd());
                }
                else{
                    best_point.iter(-1, 0);
                }
            }
            if( (best_point.getVal()<0) || !best_point.valid())
                finished=true;
            else{
                paths[i].insert(paths[i].begin()+(best_point.getP()+1), best_point.getInd()+1);
                for(auto goal=0u; goal<goals.size(); goal++){
                    if(maxValue[goal]>=0 && costsP[best_point.getInd()][goal]>=0){
                        if( costsP[best_point.getInd()][goal]<= goal_costs[goal] ){
                            //cout<<(goal_costs[goal]-costsP[best_point.getInd()][goal])<<";";
                            goal_costs[goal]=costsP[best_point.getInd()][goal];
                            //cout<<goal<<"; ";
                        }
                        goal_visited[goal]=true;
                    }
                }
                //cout<<endl;
                points_visited[best_point.getInd()]=true;
            }
            for(auto test=0u; test<goal_costs.size(); test++){
                //if(goals[test].i<100 && goals[test].j>100)
                    //cout<<goal_costs[test]<<"; ";
            }
            //cout<<endl;
            num_visited++;
            if( (num_visited==points_visited.size()) )
                finished=true;
            //points.pop_back();
        }
        clusterPts[i]=cluster_centers;
        goals_costs=goal_costs;
        goals_visited=goal_visited;
    }

    Apath path_temp;
    PointI p_0;
    cout<<"Robot 0: ";
    for(unsigned int p_i=1;p_i<paths[0].size();p_i++){
        if(paths[0][p_i-1]==0)
            p_0=pr[0].front();
        else
            p_0=clusterPts[0][paths[0][p_i-1]-1];
        cout<<p_0.i<<":"<<p_0.j<<"; ";
        //cout<<clusterPts[0][paths[0][p_i]-1].i<<":"<<clusterPts[0][paths[0][p_i]-1].j<<"; ";
        path_temp=Astar<float>(p_0, clusterPts[0][paths[0][p_i]-1], msg_rcv[E_MAP+0]);
        pr[0].insert(pr[0].end(), path_temp.points.begin(), path_temp.points.end());
    }
    if(paths[0].back()==0)
        cout<<pr[0].front().i<<":"<<pr[0].front().j<<"; "<<endl;
    else
        cout<<clusterPts[0][paths[0].back()-1].i<<":"<<clusterPts[0][paths[0].back()-1].j<<"; "<<endl;
//    for(unsigned int p_i=1;p_i<paths[1].size();p_i++){
//        if(paths[1][p_i-1]==0)
//            p_0=pr[1].front();
//        else
//            p_0=clusterPts[1][paths[1][p_i-1]-1];
//        path_temp=Astar<float>(p_0, clusterPts[1][paths[1][p_i-1]], msg_rcv[E_MAP+1]);
//        pr[1].insert(pr[1].end(), path_temp.points.begin(), path_temp.points.end());
//    }
    path_0.poses.clear();
    for(unsigned int p_i=0;p_i<pr[0].size();p_i++){
        p_temp.pose.position=convertI2W(pr[0][p_i], res);
        path_0.poses.push_back(p_temp);
    }
//    path_1.poses.clear();
//    for(unsigned int p_i=0;p_i<pr[1].size();p_i++){
//        p_temp.pose.position=convertI2W(pr[1][p_i], res);
//        path_1.poses.push_back(p_temp);
//    }
    pub1.publish(path_0);
    //pub2.publish(path_1);



    ROS_INFO("%d %d %d %d", count[0], count[1], count[2], count[3]);

    ros::Duration diff = ros::Time::now() - t0;
    ROS_INFO("Time for brute force of PAstar: %f; cost: %f", diff.toSec(), evaluate(costsMs,feasible_goals,paths,goals_costs,goals_visited));

    /////////////////////////// METHOD 2 ///////////////////////////////////////

    cout<<"----> Method 2 (Test Seq):"<<endl;
    t0=ros::Time::now();

    cout<<"Robot 0: ";

    vector<vector<PointI> > prTest=pr;
    prTest[0].clear();
    prTest[0].push_back(pr[0].front());
    vector<vector<int> > pathTest(2, vector<int>(1, 0));
    pathTest[0].push_back(1);
    pathTest[0].push_back(2);
    for(unsigned int p_i=1;p_i<pathTest[0].size();p_i++){
        if(pathTest[0][p_i-1]==0)
            p_0=pr[0].front();
        else{
            if((pathTest[0][p_i-1]-1)<0 || (pathTest[0][p_i-1]-1)>=(int)clusterPts[0].size() )
                break;
            p_0=clusterPts[0][pathTest[0][p_i-1]-1];
        }
        if((pathTest[0][p_i]-1)<0 || (pathTest[0][p_i]-1)>=(int)clusterPts[0].size() )
            break;
        path_temp=Astar<float>(p_0, clusterPts[0][pathTest[0][p_i]-1], msg_rcv[E_MAP+0]);
        prTest[0].insert(prTest[0].end(), path_temp.points.begin(), path_temp.points.end());
    }
    path_2.poses.clear();
    for(unsigned int p_i=0;p_i<prTest[0].size();p_i++){
        p_temp.pose.position=convertI2W(prTest[0][p_i], res);
        path_2.poses.push_back(p_temp);
    }
    pub3.publish(path_2);

    diff = ros::Time::now() - t0;
    ROS_INFO("Time for planning with Test Sequence: %f; cost: %f", diff.toSec(), evaluate(costsMs,costsPs,feasible_goals,pathTest));


    /////////////////////////// METHOD 3 ///////////////////////////////////////

    cout<<"----> Method 3 (Real BF):"<<endl;
    t0=ros::Time::now();

    vector<int> cl_seq;
    for(int i=1; i<=(int)clusterPts[0].size(); i++){
        cl_seq.push_back(i);
    }
    Sequence seq=permute(cl_seq, vector<int>(0));
    FindMin<float, vector<int> > min_seq;
    vector<vector<int> > pathBFseq;
    for(unsigned int i=0; i<seq.seq.size(); i++){
        if(i==0){
            pathBFseq.push_back(seq.seq[i]);
            pathBFseq.push_back(seq.rem[i]);
        }
        else{
            pathBFseq[0]=seq.seq[i];
            pathBFseq[1]=seq.rem[i];
        }
        float cost=evaluate(costsMs,costsPs,feasible_goals,pathBFseq,min_seq.valid()?min_seq.getVal():-1);
        if(cost<0)
            continue;
        min_seq.iter(cost,seq.seq[i]);
    }

    cout<<"Robot 0: ";

    prTest=pr;
    prTest[0].clear();
    prTest[0].push_back(pr[0].front());
    vector<vector<int> > pathBF(2, vector<int>(1, 0));
    pathBF[0]=min_seq.getP();
    pathBF[0].insert(pathBF[0].begin(), 0);
    for(unsigned int p_i=1;p_i<pathBF[0].size();p_i++){
        if(pathBF[0][p_i-1]==0)
            p_0=pr[0].front();
        else{
            if((pathBF[0][p_i-1]-1)<0 || (pathBF[0][p_i-1]-1)>=(int)clusterPts[0].size() )
                continue;
            p_0=clusterPts[0][pathBF[0][p_i-1]-1];
        }
        if((pathBF[0][p_i]-1)<0 || (pathBF[0][p_i]-1)>=(int)clusterPts[0].size() )
            continue;
        path_temp=Astar<float>(p_0, clusterPts[0][pathBF[0][p_i]-1], msg_rcv[E_MAP+0]);
        prTest[0].insert(prTest[0].end(), path_temp.points.begin(), path_temp.points.end());
    }
    path_3.poses.clear();
    for(unsigned int p_i=0;p_i<prTest[0].size();p_i++){
        p_temp.pose.position=convertI2W(prTest[0][p_i], res);
        path_3.poses.push_back(p_temp);
    }
    pub4.publish(path_3);

    diff = ros::Time::now() - t0;
    ROS_INFO("Time for planning with real BF: %f; cost: %f", diff.toSec(), min_seq.getVal());


    /////////////////////////// METHOD 4 ///////////////////////////////////////

    cout<<"----> Method 4 (Vis):"<<endl;
    t0=ros::Time::now();

    diff = ros::Time::now() - t0;
    ROS_INFO("Time for planning with Visibity Maps: %f", diff.toSec());

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
          point.pose.position=convertI2W(goals[i], res);
          point.id = i;
          points.markers.push_back(point);
        }
        pub_markers.publish(points);
    }
}


float Multirobotplannersensing::evaluate(vector<vector<vector<float> > > costM, vector<bool> feasible_goals, vector<vector<int> > paths, vector<float> goal_costs, vector<bool> goal_visited, float th){
    bool stop_th=false;
    if(th>=0)
        stop_th=true;
    float sum=0;
    for(unsigned int pp=0; pp<paths.size(); pp++){
        bool resolve=false;
        if(paths[pp].empty())
            resolve=true;
        else if(paths[pp][0]!=0)
            resolve=true;
        if(resolve)
            paths[pp].insert(paths[pp].begin(), 0);
    }
    for(unsigned int g=0; g<feasible_goals.size(); g++){
        if(feasible_goals[g] && !goal_visited[g]){
            //for(int abc=0; abc<feasible_goals.size();abc++){
            //    cout<<g<<"-"<<feasible_goals[g]<<":"<<goal_visited[g]<<"; ";
            //}
            return -1;
        }
        for(auto region=0u; region<goalsRegionsIds[g].size(); region++){
            sum+=(goal_costs[g])/goalsInRegions[goalsRegionsIds[g][region]].size();
            if(stop_th)
                if(sum>th)
                    return sum;
        }
    }
    for(unsigned int i=0; i<paths.size(); i++){
        for(unsigned int pt=1; pt<paths[i].size(); pt++){
            if( (paths[i][pt-1]<0) || (paths[i][pt-1]>=(int)costM[i].size()) )
                return -1;
            if( (paths[i][pt]<0) || (paths[i][pt]>=(int)costM[i].size()) )
                return -1;
            sum+=costM[i][paths[i][pt]][paths[i][pt-1]];
            if(stop_th)
                if(sum>th)
                    return sum;
        }
    }
    return sum;
}

float Multirobotplannersensing::evaluate(vector<vector<vector<float> > > costM, vector<vector<vector<float> > > costP, vector<bool> feasible_goals, vector<vector<int> > paths, float th){
    vector<float> goal_costs(feasible_goals.size(), -1);
    vector<bool> goal_visited(feasible_goals.size(), false);
    for(unsigned int pp=0; pp<paths.size(); pp++){
        bool resolve=false;
        if(paths[pp].empty())
            resolve=true;
        else if(paths[pp][0]!=0)
            resolve=true;
        if(resolve)
            paths[pp].insert(paths[pp].begin(), 0);
    }
    for(unsigned int i=0; i<paths.size(); i++){
        for(unsigned int pt=1; pt<paths[i].size(); pt++){
            for(unsigned int g=0; g<feasible_goals.size(); g++){
                if(!feasible_goals[g])
                    continue;
                if( ((paths[i][pt]-1)<0) || ((paths[i][pt]-1)>=(int)costP[i].size()) )
                    return -1;
                if( costP[i][paths[i][pt]-1][g]>=0 ){
                    goal_visited[g]=true;
                    if(goal_costs[g]<0)
                        goal_costs[g]=costP[i][paths[i][pt]-1][g];
                    else if(goal_costs[g]>costP[i][paths[i][pt]-1][g])
                        goal_costs[g]=costP[i][paths[i][pt]-1][g];
                }
            }
        }
    }
    return evaluate(costM, feasible_goals, paths, goal_costs, goal_visited, th);
}

float Multirobotplannersensing::costSensing(float K, float dist){
    return K*dist*dist;
}

int main(int argc, char **argv){ 
  ros::init(argc, argv, "multi_sensing_planning");
  ros::NodeHandle nh("~");
  Multirobotplannersensing planner(nh);
  ros::Rate loop_rate(10);

  vector<int> xxx; xxx.push_back(3); xxx.push_back(1); xxx.push_back(5);
  Sequence x;
  cout<<"------ P0 ------"<<endl;
  x=permutations(xxx,0);
  x.print();
  cout<<"------ P1 ------"<<endl;
  x=permutations(xxx,1);
  x.print();
  cout<<"------ P2 ------"<<endl;
  x=permutations(xxx,2);
  x.print();

  cout<<"------ P3 ------"<<endl;
  x=permutations(xxx,3);
  x.print();
  cout<<"------ P4 ------"<<endl;
  x=permutations(xxx,4);
  x.print();

  cout<<"------ C0 ------"<<endl;
  x=combinations(xxx,0);
  x.print();
  cout<<"------ C1 ------"<<endl;
  x=combinations(xxx,1);
  x.print();
  cout<<"------ C2 ------"<<endl;
  x=combinations(xxx,2);
  x.print();

  cout<<"------ C3 ------"<<endl;
  x=combinations(xxx,3);
  x.print();
  cout<<"------ C4 ------"<<endl;
  x=combinations(xxx,4);
  x.print();

  vector<int> gc; gc.push_back(1); gc.push_back(2);
  vector<int> g1; g1.push_back(11); g1.push_back(12);
  vector<int> g2; g2.push_back(21); g2.push_back(22);
  cout<<"------ Comb ------"<<endl;
  x=combine(g1,g2,gc);
  x.print();
  cout<<"------ Perm ------"<<endl;
  x=permute(g1,g2);
  x.print();
  cout<<"------ Test ------"<<endl;
  x=permute(g1,vector<int>(0));
  x.print();

  while (ros::ok()){
    ros::spinOnce();
    planner.plan();
    planner.publish();
    loop_rate.sleep();
  }
  return 0;
}
