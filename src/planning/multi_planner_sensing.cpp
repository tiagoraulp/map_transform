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
#include "ray.hpp"
#include "combinatorics.hpp"
#include <cmath>

#define V_MAP 0
#define E_MAP 2
#define O_MAP 4

using namespace std;

#define LAMBDA 0.04

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
    ros::Publisher pub1, pub2, pub3, pub4, pub5, pub6, pub7, pub8, pub9;
    ros::Publisher pub_markers, pub_rob_markers;
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
    vector<vector<vector<vector<PAresult> > > > vis_responses;
    vector<vector<bool> > papositive;
    vector<vector<PointI> > crits;
    vector<vector<PointI> > critsVM;
    vector<vector<PointI> > critsPA;
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
    float costSensingSqDist(float K, float dist);
    FindMax<float, unsigned int> iterate(vector<vector<vector<float> > > costsM, vector<vector<vector<float> > > costsP, vector<float> goal_costs, vector<bool> goal_visited, vector<vector<int> > paths, vector<vector<bool> > points_visited, vector<vector<float> > newMaxValue, float max_dist, float multi_levels, unsigned int& robot,  float& n,int level=2);
public:
    Multirobotplannersensing(ros::NodeHandle nh): nh_(nh){
        pub1 = nh_.advertise<nav_msgs::Path>("pathH_0", 1,true);
        pub2 = nh_.advertise<nav_msgs::Path>("pathH_1", 1,true);
        pub3 = nh_.advertise<nav_msgs::Path>("pathTest", 1,true);
        pub4 = nh_.advertise<nav_msgs::Path>("pathBF_0", 1,true);
        pub5 = nh_.advertise<nav_msgs::Path>("pathBF_1", 1,true);
        pub6 = nh_.advertise<nav_msgs::Path>("pathVis_0", 1,true);
        pub7 = nh_.advertise<nav_msgs::Path>("pathVis_1", 1,true);
        pub8 = nh_.advertise<nav_msgs::Path>("pathVisTest_0", 1,true);
        pub9 = nh_.advertise<nav_msgs::Path>("pathVisTest_1", 1,true);
        pub_markers = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1,true);
        pub_rob_markers = nh_.advertise<visualization_msgs::MarkerArray>("visualization_robot_marker_array", 1,true);
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
        vis_responses.resize(2);
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

        crits.resize(2, vector<PointI>(0));
        critsVM.resize(2, vector<PointI>(0));
        critsPA.resize(2, vector<PointI>(0));
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
    return map_rcv[O_MAP] && map_rcv[V_MAP] && map_rcv[V_MAP+1] && map_rcv[E_MAP] && map_rcv[E_MAP+1] && graph_rcv[0] && graph_rcv[1];
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
    papositive=vector<vector<bool> >(2, vector<bool>(goals.size(), true));
}

void Multirobotplannersensing::plan(void){
    if(!allAvailable() || !pl ){
        return;
    }
    nav_msgs::Path path_0,path_1, path_2, path_3, path_4, path_5, path_6;

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
    path_5=path_0;
    path_6=path_0;

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

    /////////////////////////// METHOD 0 ///////////////////////////////////////

    cout<<"----> Determination with PA*:"<<endl;
    ros::Time t0=ros::Time::now();

    bf_responses[0].assign(goals.size(), vector<PAresult>(0, PAresult()));
    bf_responses[1].assign(goals.size(), vector<PAresult>(0, PAresult()));

    vector<vector<int> > paths(2, vector<int>(1, 0));
    vector<vector<PointI> > clusterPts(2, vector<PointI>(0));

    vector<vector<float> > newMaxValue(2, vector<float>(goals.size(), -1));

    vector<vector<vector<float> > > costsMs(2);
    vector<vector<vector<float> > > costsPs(2);
    vector<bool> feasible_goals(goals.size(), false);
    vector<float> goals_costs;
    vector<bool> goals_visited;

    vector<int> goalsss;

    vector<float> maxValue(goals.size(), -1);
    float max_dist=-1;

    vector<vector<bool> > points_visited(2, vector<bool>(0));

    papositive=vector<vector<bool> >(2, vector<bool>(goals.size(), true));


    vector<int> count(4, 0);
    critsPA.assign(2, vector<PointI>(0));
    for(int i=0; i<2; i++){
    //for(int i=0; i<1; i++){
        cout<<"Robot "<<i<<endl;
        PAstar pastar(infl[i], defl[i]);
        //cout<<"BLALBALBLALB: "<<defl[i]<<endl;
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
            if(n>0)
                critsPA[i].push_back(p0);
            int curr_cluster=clusterpointsID.back();
            clusterpoints.pop_back();
            clusterpointsID.pop_back();
            if((curr_cluster+1)>=(int)clusterVisitOrder.size()){
                clusterVisitOrder.resize(cluster_centers.size()+1, -1);
            }
            clusterVisitOrder[curr_cluster+1]=n;
            cout<<"Robot "<<i<<"; From Point "<<n<<"; "<<p0.i<<" "<<p0.j<<"; order: "<<curr_cluster<<endl;
            n++;
            if(p0.i==71 && p0.j==76 && i==0)
                goalsss.push_back(curr_cluster);
            if(p0.i==71 && p0.j==94 && i==0)
                goalsss.push_back(curr_cluster);

            vector<cv::Point> points(0);
            vector<vector<vector<int> > > pointGoals(width, vector<vector<int> >(height,vector<int>(0)));
            for(unsigned int g=0; g<goals.size(); g++){
                PAresult paresult;
                if(papositive[i][g]){
                    PApath path=pastar.run(p0, goals[g], LAMBDA, true);
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
                        papositive[i][g]=false;
                        count[i+2]++;
                    }
                }
                else{
                    paresult.cost=-1;
                    paresult.p0=p0;
                    paresult.costM=-1;
                    paresult.costP=-1;
                }
                bf_responses[i][g].push_back(paresult);
            }
            vector<vector<bool> > points_map(or_map.rows, vector<bool>(or_map.cols, false));
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
                if( (closest_cluster.getVal()>(threshold*threshold)) || !closest_cluster.valid() ){
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
                    if(!points_map[clusters[cc][pp].x][clusters[cc][pp].y]){
                        for(unsigned int gg=0; gg<pointGoals[clusters[cc][pp].x][clusters[cc][pp].y].size(); gg++){
                            goalsInClusters[active_cluster][curr_cluster+1][pointGoals[clusters[cc][pp].x][clusters[cc][pp].y][gg]]=true;
                        }
                        points_map[clusters[cc][pp].x][clusters[cc][pp].y]=true;
                    }
                }
            }
        }
        vector<vector<float> > costsM(cluster_centers.size()+1,vector<float>(cluster_centers.size()+1,-1));
        vector<vector<float> > costsP(cluster_centers.size(),vector<float>(goals.size(),-1));
        for(unsigned int pt=0; pt<cluster_centers.size();pt++){
            for(unsigned int other=0; other<=cluster_centers.size(); other++){
                float sum=0, number=0;
                if(other<goalsInClusters[pt].size() ){
                    for(unsigned int goal=0; goal<goalsInClusters[pt][other].size(); goal++){
                        if( goalsInClusters[pt][other][goal] ){
                            float new_cost_p=-1;
                            if(bf_responses[i][goal][clusterVisitOrder[other]].cost>=0){
                                sum+=bf_responses[i][goal][clusterVisitOrder[other]].costM;
                                number++;
                                new_cost_p=bf_responses[i][goal][clusterVisitOrder[other]].costP;
                            }
                            else{
                                new_cost_p=-1;
                                cout<<"Aqui ===: "<<goal<<" ("<<pt<<"; "<<other<<")"<<endl;
                            }
                            if( costsP[pt][goal]<0 && new_cost_p>=0 ){
                                feasible_goals[goal]=true;
                                newMaxValue[i][goal]=costSensing(LAMBDA,(float)defl[i]);
                                maxValue[goal]=costSensing(LAMBDA,(float)max(defl[0], defl[1]));
                                costsP[pt][goal]=new_cost_p;
                                //if( maxValue[goal]<costsP[pt][goal] ){
                                //    maxValue[goal]=costsP[pt][goal];
                                //}
                            }
                            else if(new_cost_p>=0 && (costsP[pt][goal]>new_cost_p) ){
                                costsP[pt][goal]=new_cost_p;
                                //if( maxValue[goal]<costsP[pt][goal] ){
                                //    maxValue[goal]=costsP[pt][goal];
                                //}
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
            for(unsigned int goal=0; goal<goals.size(); goal++){
                if( (costsP[pt][goal]<0) && papositive[i][goal]){
                    if( ( (cluster_centers[pt].i-goals[goal].i)*(cluster_centers[pt].i-goals[goal].i)+(cluster_centers[pt].j-goals[goal].j)*(cluster_centers[pt].j-goals[goal].j) )<=(defl[i]*defl[i]) ){
                        if(raytracing(or_map, cluster_centers[pt].i, cluster_centers[pt].j, goals[goal].i, goals[goal].j, true)){
                            feasible_goals[goal]=true;
                            newMaxValue[i][goal]=costSensing(LAMBDA,(float)defl[i]);
                            maxValue[goal]=costSensing(LAMBDA,(float)max(defl[0], defl[1]));
                            costsP[pt][goal]=costSensingSqDist(LAMBDA,( (cluster_centers[pt].i-goals[goal].i)*(cluster_centers[pt].i-goals[goal].i)+(cluster_centers[pt].j-goals[goal].j)*(cluster_centers[pt].j-goals[goal].j) ));
                        }
                    }
                }
            }
        }
        //costsMs.push_back(costsM);
        //costsPs.push_back(costsP);


        costsMs[i]=costsM;
        costsPs[i]=costsP;


//        if(i>0)
//            continue;
//        vector<float> goal_costs=newMaxValue[i];//=maxValue;
//        vector<bool> goal_visited(goals.size(), false);
//        vector<PointI> points=cluster_centers;
//        vector<bool> points_visited(points.size(), false);
//        unsigned int num_visited=0;
//        bool finished=false;
//        while(!finished){
//        //while(!points.empty()){
//            float n_temp;
//            unsigned int robot=i;
//            FindMax<float, unsigned int> best_point=iterate(costsMs, costsPs, goal_costs, goal_visited, paths, points, points_visited, max_dist, robot,n_temp, 3);
//            if( (best_point.getVal()<(-max_dist*2)) || !best_point.valid())
//                finished=true;
//            else{
//                paths[i].insert(paths[i].begin()+(best_point.getP()+1), best_point.getInd()+1);
//                for(auto goal=0u; goal<goals.size(); goal++){
//                    if(maxValue[goal]>=0 && costsP[best_point.getInd()][goal]>=0){
//                        if( costsP[best_point.getInd()][goal]<= goal_costs[goal] ){
//                            goal_costs[goal]=costsP[best_point.getInd()][goal];
//                        }
//                        goal_visited[goal]=true;
//                    }
//                }
//                points_visited[best_point.getInd()]=true;
//            }
//            num_visited++;
//            if( (num_visited==points_visited.size()) )
//                finished=true;
//            //points.pop_back();
//        }


        clusterPts[i]=cluster_centers;
        points_visited[i]=vector<bool>(cluster_centers.size(), false);
        //goals_costs=goal_costs;
        //goals_visited=goal_visited;
    }

    ros::Duration diff = ros::Time::now() - t0;
    ROS_INFO("Time for PAstar determination: %f", diff.toSec());

    /////////////////////////// METHOD 1 ///////////////////////////////////////

    cout<<"----> Method 1 (Heuristic Search):"<<endl;
    t0=ros::Time::now();

    float multi_levels=regions.size()*max(max_dist, costSensing(LAMBDA,(float)max(defl[0], defl[1])) ); // ?? think, probably no need for max_dist

    //vector<float> goal_costs=newMaxValue[i];
    vector<float> goal_costs=maxValue;
    vector<bool> goal_visited(goals.size(), false);
    //vector<PointI> points=cluster_centers;
    //vector<bool> points_visited(points.size(), false);
    unsigned int size_points=points_visited[0].size()+points_visited[1].size();
    unsigned int num_visited=0;
    bool finished=false;
    while(!finished){
        float n_temp;
        unsigned int robot;
        int depth=3;
        FindMax<float, unsigned int> best_point=iterate(costsMs, costsPs, goal_costs, goal_visited, paths, points_visited, newMaxValue, max_dist, multi_levels, robot,n_temp, depth);
        if( (best_point.getVal()<(-max_dist*depth)) || !best_point.valid())
            finished=true;
        else{
            //cout<<"Choose robot "<<robot<<"; cluster point: "<<best_point.getInd()<<endl;
            paths[robot].insert(paths[robot].begin()+(best_point.getP()+1), best_point.getInd()+1);
            for(auto goal=0u; goal<goals.size(); goal++){
                if(maxValue[goal]>=0 && costsPs[robot][best_point.getInd()][goal]>=0){
                    if( costsPs[robot][best_point.getInd()][goal]<= goal_costs[goal] ){
                        goal_costs[goal]=costsPs[robot][best_point.getInd()][goal];
                    }
                    goal_visited[goal]=true;
                }
            }
            points_visited[robot][best_point.getInd()]=true;
        }
        num_visited++;
        if( (num_visited==size_points) )
            finished=true;
    }
    goals_costs=goal_costs;
    goals_visited=goal_visited;

    Apath path_temp;
    PointI p_0;
    cout<<"Robot 0: ";
    for(unsigned int p_i=1;p_i<paths[0].size();p_i++){
        cout<<paths[0][p_i]<<"; ";
        if(paths[0][p_i-1]==0)
            p_0=pr[0].front();
        else
            p_0=clusterPts[0][paths[0][p_i-1]-1];
        //cout<<p_0.i<<":"<<p_0.j<<"; ";
        //cout<<clusterPts[0][paths[0][p_i]-1].i<<":"<<clusterPts[0][paths[0][p_i]-1].j<<"; ";
        path_temp=Astar<float>(p_0, clusterPts[0][paths[0][p_i]-1], msg_rcv[E_MAP+0]);
        pr[0].insert(pr[0].end(), path_temp.points.begin(), path_temp.points.end());
    }
    //if(paths[0].back()==0)
    //    cout<<pr[0].front().i<<":"<<pr[0].front().j<<"; "<<endl;
    //else
    //    cout<<clusterPts[0][paths[0].back()-1].i<<":"<<clusterPts[0][paths[0].back()-1].j<<"; "<<endl;
    cout<<"Robot 1: ";
    for(unsigned int p_i=1;p_i<paths[1].size();p_i++){
        cout<<paths[1][p_i]<<"; ";
        if(paths[1][p_i-1]==0)
            p_0=pr[1].front();
        else
            p_0=clusterPts[1][paths[1][p_i-1]-1];
        path_temp=Astar<float>(p_0, clusterPts[1][paths[1][p_i]-1], msg_rcv[E_MAP+1]);
        pr[1].insert(pr[1].end(), path_temp.points.begin(), path_temp.points.end());
    }
    path_0.poses.clear();
    for(unsigned int p_i=0;p_i<pr[0].size();p_i++){
        p_temp.pose.position=convertI2W(pr[0][p_i], res);
        path_0.poses.push_back(p_temp);
    }
    path_1.poses.clear();
    for(unsigned int p_i=0;p_i<pr[1].size();p_i++){
        p_temp.pose.position=convertI2W(pr[1][p_i], res);
        path_1.poses.push_back(p_temp);
    }
    pub1.publish(path_0);
    pub2.publish(path_1);



    //ROS_INFO("%d %d %d %d", count[0], count[1], count[2], count[3]);

    diff = ros::Time::now() - t0;
    ROS_INFO("Time for Heuristic approach: %f; cost: %f", diff.toSec(), evaluate(costsMs,feasible_goals,paths,goals_costs,goals_visited));

    /////////////////////////// METHOD 2 ///////////////////////////////////////

    cout<<"----> Method 2 (Test Seq):"<<endl;
    t0=ros::Time::now();

    cout<<"Robot 0: ";

    vector<vector<PointI> > prTest=pr;
    prTest[0].clear();
    prTest[0].push_back(pr[0].front());
    vector<vector<int> > pathTest(2, vector<int>(1, 0));


    for(unsigned int sss=0; sss<goalsss.size(); sss++)
        pathTest[0].push_back(goalsss[sss]+1);

    //pathTest[0].push_back(2);
    //pathTest[0].push_back(2);
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

    cout<<"----> Method 3 (Real combinatorial BF):"<<endl;
    t0=ros::Time::now();

    vector<vector<int> > cl_seq(2);
    for(unsigned int robot=0; robot<paths.size(); robot++){
        for(int i=1; i<=(int)clusterPts[robot].size(); i++){
            cl_seq[robot].push_back(i);
        }
    }
    //Sequence seq=permute(cl_seq, vector<int>(0));
    Sequence seq;//=permute(cl_seq[0], cl_seq[1]);
    FindMin<float, vector<int> > min_seq;
    FindMin<float, vector<int> > min_seq2;
    vector<vector<int> > pathBFseq;
    for(unsigned int i=0; i<seq.seq.size(); i++){
        //if((i%500)==0)
        //    cout<<i<<" out of "<<seq.seq.size()<<endl;
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
        min_seq2.iter(cost,seq.rem[i]);
    }

    cout<<"Robot 0: ";

    prTest=pr;
    prTest[0].clear();
    prTest[0].push_back(pr[0].front());
    prTest[1].clear();
    prTest[1].push_back(pr[1].front());
    vector<vector<int> > pathBF(2, vector<int>(1, 0));
    pathBF[0]=min_seq.getP();
    pathBF[0].insert(pathBF[0].begin(), 0);
    pathBF[1]=min_seq2.getP();
    pathBF[1].insert(pathBF[1].begin(), 0);
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

    for(unsigned int p_i=1;p_i<pathBF[1].size();p_i++){
        if(pathBF[1][p_i-1]==0)
            p_0=pr[1].front();
        else{
            if((pathBF[1][p_i-1]-1)<0 || (pathBF[1][p_i-1]-1)>=(int)clusterPts[1].size() )
                continue;
            p_0=clusterPts[1][pathBF[1][p_i-1]-1];
        }
        if((pathBF[1][p_i]-1)<0 || (pathBF[1][p_i]-1)>=(int)clusterPts[1].size() )
            continue;
        path_temp=Astar<float>(p_0, clusterPts[1][pathBF[1][p_i]-1], msg_rcv[E_MAP+1]);
        prTest[1].insert(prTest[1].end(), path_temp.points.begin(), path_temp.points.end());
    }
    path_4.poses.clear();
    for(unsigned int p_i=0;p_i<prTest[1].size();p_i++){
        p_temp.pose.position=convertI2W(prTest[1][p_i], res);
        path_4.poses.push_back(p_temp);
    }
    pub5.publish(path_4);

    diff = ros::Time::now() - t0;
    ROS_INFO("Time for planning with real BF: %f; cost: %f", diff.toSec(), min_seq.getVal());

    /////////////////////////// METHOD 0+ ///////////////////////////////////////

    cout<<"----> Cluster Determination with Visibility:"<<endl;
    t0=ros::Time::now();


    vis_responses[0].assign(goals.size(), vector<vector<PAresult> >(0));
    vis_responses[1].assign(goals.size(), vector<vector<PAresult> >(0));
    clusterPts=vector<vector<PointI> >(2, vector<PointI>(0));
    newMaxValue=vector<vector<float> >(2, vector<float>(goals.size(), -1));
    feasible_goals=vector<bool>(goals.size(), false);
    maxValue=vector<float> (goals.size(), -1);
    max_dist=-1;
    papositive=vector<vector<bool> >(2, vector<bool>(goals.size(), true));

    float opt_dist=1.0/2.0/LAMBDA;

    crits.assign(2, vector<PointI>(0));
    critsVM.assign(2, vector<PointI>(0));
    for(unsigned int i=0; i<2; i++){
    //for(unsigned int i=0; i<1; i++){
        vector<vector<int> > crit_goals;
        vector<PointI> crit_pts;
        vector<vector<int> > crit_map(or_map.rows, vector<int>(or_map.cols,-1));
        vector<vector<int> > other_goals;
        vector<PointI> other_pts;
        vector<vector<int> > other_map(or_map.rows, vector<int>(or_map.cols,-1));
        vector<vector<vector<int> > > other_temp_map(or_map.rows, vector<vector<int> >(or_map.cols,vector<int>(0)));
        vector<cv::Point> other_temp;
        for(unsigned int goal=0; goal<goals.size(); goal++){
            int index=goals[goal].i*or_map.cols+goals[goal].j;
            if( vis_[i][index]<0 ){
                for(unsigned int crit=0; crit<crit_points[i][index].points.size(); crit++){
                    PointI critPt(crit_points[i][index].points[crit].position.x,crit_points[i][index].points[crit].position.y);
                    if( other_temp_map[critPt.i][critPt.j].empty() )
                        other_temp.push_back(cv::Point(critPt.i, critPt.j));
                    other_temp_map[critPt.i][critPt.j].push_back(goal);
                }
            }
            else{
                for(unsigned int crit=0; crit<crit_points[i][index].points.size(); crit++){
                    PointI critPt(crit_points[i][index].points[crit].position.x,crit_points[i][index].points[crit].position.y);
                    crits[i].push_back(critPt);
                    if( crit_map[critPt.i][critPt.j]<0 ){
                        crit_pts.push_back(critPt);
                        crit_map[critPt.i][critPt.j]=crit_pts.size()-1;
                        crit_goals.push_back(vector<int>(1, goal));
                    }
                    else{
                        crit_goals[crit_map[critPt.i][critPt.j]].push_back(goal);
                    }
                }
            }
        }
        vector<vector<PointI> > other_clusters;
        vector<vector<cv::Point> > groups=cluster_points(other_temp);
        for(unsigned int group=0; group<groups.size(); group++){
            vector<PointI> other_pts_temp;
            for(unsigned int pt=0; pt<groups[group].size(); pt++){
                FindMin<float, PointI> closest_other;
                for(unsigned other=0; other<other_pts_temp.size(); other++){
                    float d1=groups[group][pt].x-other_pts_temp[other].i;
                    float d2=groups[group][pt].y-other_pts_temp[other].j;
                    closest_other.iter( d1*d1+d2*d2, other_pts_temp[other]);
                }
                float threshold=opt_dist*opt_dist;
                // ////////////////////////////////////////////// what happens if ray tracing fails?????? it's not added neither creates new cluster!!
                bool new_cluster=false;
                if(closest_other.valid() && (closest_other.getVal()<=(threshold*threshold)) ){
                    if( raytracing(or_map, closest_other.getP().i, closest_other.getP().j,groups[group][pt].x, groups[group][pt].y,true)){
                        other_goals[other_map[closest_other.getP().i][closest_other.getP().j]].insert(other_goals[other_map[closest_other.getP().i][closest_other.getP().j]].begin(),
                            other_temp_map[groups[group][pt].x][groups[group][pt].y].begin(), other_temp_map[groups[group][pt].x][groups[group][pt].y].end());
                        other_clusters[other_map[closest_other.getP().i][closest_other.getP().j]].push_back(PointI(groups[group][pt].x,groups[group][pt].y));
                    }
                    else{
                        new_cluster=true;
                    }
                }
                else{
                    new_cluster=true;
                }
                if(new_cluster){
                    other_pts_temp.push_back(PointI(groups[group][pt].x,groups[group][pt].y));
                    other_pts.push_back(PointI(groups[group][pt].x,groups[group][pt].y));
                    other_map[groups[group][pt].x][groups[group][pt].y]=other_pts.size()-1;
                    other_goals.push_back(other_temp_map[groups[group][pt].x][groups[group][pt].y]);
                    other_clusters.push_back(vector<PointI>(1,PointI(groups[group][pt].x,groups[group][pt].y)));
                }
            }
        }
        vector<vector<float> > other_goals_cost(goals.size(), vector<float>(other_pts.size(),-1));
        for(unsigned int pt=0; pt<other_goals.size(); pt++){
            for(unsigned int goal=0; goal<other_goals[pt].size(); goal++){
                int goalOr=other_goals[pt][goal];
                if(other_goals_cost[goalOr][pt]<0)
                    other_goals_cost[goalOr][pt]=costSensingSqDist(LAMBDA, (other_pts[pt].i-goals[goalOr].i)*(other_pts[pt].i-goals[goalOr].i)+(other_pts[pt].j-goals[goalOr].j)*(other_pts[pt].j-goals[goalOr].j));
            }
            float posi=0, posj=0;
            for(unsigned int other=0; other<other_clusters[pt].size();other++){
                posi+=other_clusters[pt][other].i;
                posj+=other_clusters[pt][other].j;
            }
            posi/=(float)other_clusters[pt].size();
            posj/=(float)other_clusters[pt].size();
            posi=round(posi);
            posj=round(posj);
            FindMin<unsigned long int, PointI> med;
            for(unsigned int pp=0; pp<other_clusters[pt].size();pp++){
                med.iter(other_clusters[pt][pp].diff2(PointI(posi,posj)),other_clusters[pt][pp]);
            }
            other_pts[pt]=med.getP();
        }

        cout<<"Robot "<<i<<endl;
        vector<PointI> clusterpoints(0);
        vector<int> clusterpointsID(0);
        clusterpoints.push_back(pr[i].front());
        clusterpointsID.push_back(-1);
        vector<int> clusterVisitOrder(0);
        vector<PointI> cluster_centers(0);
        vector<vector<vector<bool> > > goalsInClusters(0);
        vector<vector<vector<vector<int> > > > goalsInClustersRun(0);
        int n=0;
        while(!clusterpoints.empty()){
            PointI p0=clusterpoints.back();
            if(n>0)
                critsVM[i].push_back(p0);
            int curr_cluster=clusterpointsID.back();
            clusterpoints.pop_back();
            clusterpointsID.pop_back();
            if((curr_cluster+1)>=(int)clusterVisitOrder.size()){
                clusterVisitOrder.resize(cluster_centers.size()+1, -1);
            }
            cout<<"Robot "<<i<<"; From Point "<<n<<"; "<<p0.i<<" "<<p0.j<<"; order: "<<curr_cluster<<endl;
            clusterVisitOrder[curr_cluster+1]=n;
            n++;
            vector<cv::Point> points(0);
            vector<vector<Apath> > apath_map(or_map.rows, vector<Apath>(or_map.cols, Apath()));
            vector<vector<vector<int> > > pointGoals(or_map.rows, vector<vector<int> >(or_map.cols,vector<int>(0)));
            vector<vector<vector<int> > > pointGoalsRun(or_map.rows, vector<vector<int> >(or_map.cols,vector<int>(0)));
            for(unsigned int goal=0; goal<goals.size(); goal++){
                vis_responses[i][goal].resize(n,vector<PAresult>(0));
                int index=goals[goal].i*or_map.cols+goals[goal].j;
                if(crit_points[i][index].points.empty()){
                    papositive[i][goal]=false;
                }
                PAresult paresult;
                if(papositive[i][goal]){
                    //if(crit_points[i][index].points.size()>2)
                    //    cout<<"OOOOOHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH"<<endl;
                    int index=goals[goal].i*or_map.cols+goals[goal].j;
                    if( vis_[i][index]<0 ){
                        for(unsigned int crit=0; crit<other_goals_cost[goal].size(); crit++){
                            if(other_goals_cost[goal][crit]<0){
                                continue;
                            }
                            else{
                                PointI critPt=other_pts[crit];
                                Apath path;
                                if( apath_map[critPt.i][critPt.j].cost<0 ){
                                    path=Astar<float>(p0, critPt, msg_rcv[E_MAP+i]);
                                    path.points.insert(path.points.begin(),p0);
                                    apath_map[critPt.i][critPt.j]=path;
                                }
                                else{
                                    path=apath_map[critPt.i][critPt.j];
                                }
                                paresult.p0=p0;
                                paresult.perc_pt=path.points.back();
                                paresult.costP=other_goals_cost[goal][crit];
                                paresult.costM=path.cost;
                                //float offset=sqrt( (goals[goal].i-critPt.i)*(goals[goal].i-critPt.i)+(goals[goal].j-critPt.j)*(goals[goal].j-critPt.j) );
                                float offset=0;
                                //float margin=max(opt_dist-offset,(float)0.0);
                                float margin=max(2*opt_dist-offset,(float)0.0);
                                int begining=max((int)path.points.size()-1-((int)ceil(margin)), 0);
                                //if(n==1 && goal==0 && crit==0)
                                //    cout<<"Margin: "<<margin<<"; Offset: "<<offset<<"; Begin: "<<begining<<"; Test: "<<(begining!=((int)path.points.size()-1))<<endl;
                                if(begining!=((int)path.points.size()-1)){
                                    bool found_end=false;
                                    float diff=0;
                                    for(int it=begining; it<((int)path.points.size()); it++){
                                        //if(n==1 && goal==0 && crit==0)
                                        //    cout<<"it: "<<it<<" out of "<<(path.points.size()-1)<<endl;
                                        if(!found_end){
                                            float sens_dist=float((goals[goal].i-path.points[it].i)*(goals[goal].i-path.points[it].i)+(goals[goal].j-path.points[it].j)*(goals[goal].j-path.points[it].j));
                                            //if(n==1 && goal==0 && crit==0)
                                            //    cout<<"Sens dist: "<<sens_dist<<endl;
                                            if( sens_dist <=(opt_dist*opt_dist) ){
                                                //if(n==1 && goal==0 && crit==0)
                                                //    cout<<"Radius: True"<<endl;
                                                if(raytracing(or_map, path.points[it].i, path.points[it].j, goals[goal].i, goals[goal].j, true)){
                                                    //if(n==1 && goal==0 && crit==0)
                                                    //    cout<<"RayTrace: True"<<endl;
                                                    paresult.perc_pt=path.points[it];
                                                    paresult.costP=costSensingSqDist(LAMBDA, sens_dist);
                                                    found_end=true;
                                                }
                                            }
                                        }
                                        else{
                                            if( (it-1)<0 )
                                                continue;
                                            else{
                                                int num=abs(path.points[it].i-path.points[it-1].i)+abs(path.points[it].j-path.points[it-1].j);
                                                diff+=(num==2)?1.41421356237:1;
                                            }
                                        }
                                    }
                                    paresult.costM=path.cost-diff;
                                }
                                paresult.cost=paresult.costM+paresult.costP;
                                //cout<<i<<"; "<<goal<<"; "<<n-1<<"; "<<vis_responses[i][goal][n-1].size()<<" -- "<<vis_responses[0][0][0].size()<<endl;
                                vis_responses[i][goal][n-1].push_back(paresult);

                                points.push_back(cv::Point(paresult.perc_pt.i,paresult.perc_pt.j));
                                pointGoals[points.back().x][points.back().y].push_back(goal);
                                pointGoalsRun[points.back().x][points.back().y].push_back(vis_responses[i][goal][n-1].size()-1);
                            }
                        }
                    }
                    else{
                        for(unsigned int crit=0; crit<crit_points[i][index].points.size(); crit++){
                            PointI critPt(crit_points[i][index].points[crit].position.x,crit_points[i][index].points[crit].position.y);
                            Apath path;
                            if( apath_map[critPt.i][critPt.j].cost<0 ){
                                path=Astar<float>(p0, critPt, msg_rcv[E_MAP+i]);
                                path.points.insert(path.points.begin(),p0);
                                apath_map[critPt.i][critPt.j]=path;
                            }
                            else{
                                path=apath_map[critPt.i][critPt.j];
                            }
                            float offset=sqrt( (goals[goal].i-critPt.i)*(goals[goal].i-critPt.i)+(goals[goal].j-critPt.j)*(goals[goal].j-critPt.j) );
                            //if( vis_[i][index]<0 ){
                            //    offset=0;
                            //}
                            //else{
                            //    offset=sqrt( (goals[goal].i-critPt.i)*(goals[goal].i-critPt.i)+(goals[goal].j-critPt.j)*(goals[goal].j-critPt.j) );
                            //}
                            paresult.p0=p0;
                            paresult.perc_pt=path.points.back();
                            paresult.costP=costSensing(LAMBDA, offset);
                            paresult.costM=path.cost;
                            float margin=max(opt_dist-offset,(float)0.0);
                            int begining=max((int)path.points.size()-1-((int)ceil(margin)), 0);
                            //if(n==1 && goal==0 && crit==0)
                            //    cout<<"Margin: "<<margin<<"; Offset: "<<offset<<"; Begin: "<<begining<<"; Test: "<<(begining!=((int)path.points.size()-1))<<endl;
                            if(begining!=((int)path.points.size()-1)){
                                bool found_end=false;
                                float diff=0;
                                for(int it=begining; it<((int)path.points.size()); it++){
                                    //if(n==1 && goal==0 && crit==0)
                                    //    cout<<"it: "<<it<<" out of "<<(path.points.size()-1)<<endl;
                                    if(!found_end){
                                        float sens_dist=float((goals[goal].i-path.points[it].i)*(goals[goal].i-path.points[it].i)+(goals[goal].j-path.points[it].j)*(goals[goal].j-path.points[it].j));
                                        //if(n==1 && goal==0 && crit==0)
                                        //    cout<<"Sens dist: "<<sens_dist<<endl;
                                        if( sens_dist <=(opt_dist*opt_dist) ){
                                            //if(n==1 && goal==0 && crit==0)
                                            //    cout<<"Radius: True"<<endl;
                                            if(raytracing(or_map, path.points[it].i, path.points[it].j, goals[goal].i, goals[goal].j, true)){
                                                //if(n==1 && goal==0 && crit==0)
                                                //    cout<<"RayTrace: True"<<endl;
                                                paresult.perc_pt=path.points[it];
                                                paresult.costP=costSensingSqDist(LAMBDA, sens_dist);
                                                found_end=true;
                                            }
                                        }
                                    }
                                    else{
                                        if( (it-1)<0 )
                                            continue;
                                        else{
                                            int num=abs(path.points[it].i-path.points[it-1].i)+abs(path.points[it].j-path.points[it-1].j);
                                            diff+=(num==2)?1.41421356237:1;
                                        }
                                    }
                                }
                                paresult.costM=path.cost-diff;
                            }
                            else{
                                //cout<<"New!"<<endl;
                                float diff=0;
                                FindMin<float, float, float> new_perc_pt;
                                for(int it=((int)path.points.size()-2); it>=max((int)path.points.size()-2-2*infl[i],0); it--){
                                    float sens_dist=float((goals[goal].i-path.points[it].i)*(goals[goal].i-path.points[it].i)+(goals[goal].j-path.points[it].j)*(goals[goal].j-path.points[it].j));
                                    float newP=costSensingSqDist(LAMBDA, sens_dist);
                                    int num=abs(path.points[it].i-path.points[it+1].i)+abs(path.points[it].j-path.points[it+1].j);
                                    float motion=(num==2)?1.41421356237:1;
                                    diff+=motion;
                                    //cout<<newP<<" sens; "<<(paresult.costP-newP)<<" loss; "<<motion<<" gain; "<<endl;
                                    if( (-diff+newP-paresult.costP) < 0 ){
                                        if(raytracing(or_map, path.points[it].i, path.points[it].j, goals[goal].i, goals[goal].j, true)){
                                            new_perc_pt.iter(paresult.costM-diff+newP,paresult.costM-diff, newP);
                                            continue;
                                        }
                                        else
                                            break;
                                    }
                                    else{
                                        new_perc_pt.iter(-1,-1,-1);
                                    }
                                }
                                if(new_perc_pt.valid() && (new_perc_pt.getVal()>=0) ){
                                    paresult.perc_pt=path.points[(int)path.points.size()-2-new_perc_pt.getInd()];
                                    paresult.costM=new_perc_pt.getP();
                                    paresult.costP=new_perc_pt.getOP();
                                }
                            }
                            paresult.cost=paresult.costM+paresult.costP;
                            //cout<<i<<"; "<<goal<<"; "<<n-1<<"; "<<vis_responses[i][goal][n-1].size()<<" -- "<<vis_responses[0][0][0].size()<<endl;
                            vis_responses[i][goal][n-1].push_back(paresult);

                            points.push_back(cv::Point(paresult.perc_pt.i,paresult.perc_pt.j));
                            pointGoals[points.back().x][points.back().y].push_back(goal);
                            pointGoalsRun[points.back().x][points.back().y].push_back(crit);

                        }
                    }
                }
                else{
                    paresult.cost=-1;
                    paresult.p0=p0;
                    paresult.costM=-1;
                    paresult.costP=-1;
                    //cout<<i<<"; "<<goal<<"; "<<n-1<<"; "<<vis_responses[i][goal][n-1].size()<<endl;
                    vis_responses[i][goal][n-1].push_back(paresult);
                }

                //cout<<vis_responses[i][goal][n-1][0].cost<<endl;
            }
            vector<vector<bool> > points_map(or_map.rows, vector<bool>(or_map.cols, false));
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
                    goalsInClustersRun.push_back(vector<vector<vector<int> > >(cluster_centers.size()+1, vector<vector<int> >(goals.size(), vector<int>(0))));
                    active_cluster=goalsInClusters.size()-1;
                    //cout<<"T1; "<<goalsInClustersRun[0][0][0].size()<<endl;
                }
                else{
                    active_cluster=closest_cluster.getInd();
                }
                if((curr_cluster+1)>=(int)goalsInClusters[active_cluster].size()){
                    goalsInClusters[active_cluster].resize(cluster_centers.size()+1,vector<bool>(goals.size(), false));
                    goalsInClustersRun[active_cluster].resize(cluster_centers.size()+1,vector<vector<int> >(goals.size(),vector<int>(0) ));
                    //cout<<"T2; "<<goalsInClustersRun[0][0][0].size()<<endl;
                }
                for(unsigned int pp=0; pp<clusters[cc].size();pp++){
                    if(!points_map[clusters[cc][pp].x][clusters[cc][pp].y]){
                        for(unsigned int gg=0; gg<pointGoals[clusters[cc][pp].x][clusters[cc][pp].y].size(); gg++){
                            goalsInClusters[active_cluster][curr_cluster+1][pointGoals[clusters[cc][pp].x][clusters[cc][pp].y][gg]]=true;
                            goalsInClustersRun[active_cluster][curr_cluster+1][pointGoals[clusters[cc][pp].x][clusters[cc][pp].y][gg]].push_back(pointGoalsRun[clusters[cc][pp].x][clusters[cc][pp].y][gg]);
                        //cout<<"T3; "<<goalsInClustersRun[0][0][0].size()<<endl;
                        }
                        points_map[clusters[cc][pp].x][clusters[cc][pp].y]=true;
                    }
                }
            }
        }
        cout<<"Processing "<<i<<endl;
        vector<vector<float> > costsM(cluster_centers.size()+1,vector<float>(cluster_centers.size()+1,-1));
        vector<vector<float> > costsP(cluster_centers.size(),vector<float>(goals.size(),-1));
        for(unsigned int pt=0; pt<cluster_centers.size();pt++){
            for(unsigned int other=0; other<=cluster_centers.size(); other++){
                float sum=0, number=0;
                if(other<goalsInClusters[pt].size() ){
                    for(unsigned int goal=0; goal<goalsInClusters[pt][other].size(); goal++){
                        if( goalsInClusters[pt][other][goal] ){
                            //if(goal==0 && other==0)
                            //    cout<<goalsInClustersRun[pt][other][goal].size()<<endl;
                            //if(goal==1 && other==1)
                            //    cout<<goalsInClustersRun[pt][other][goal].size()<<endl;
                            for(unsigned int crit=0; (crit<goalsInClustersRun[pt][other][goal].size()) ; crit++){
                                //cout<<"pt "<<pt<<";other "<<other<<"; goal "<<goal<<" of "<<goals.size()<<"; crit "<<crit<<"; limit "<<goalsInClustersRun[pt][other][goal].size()<<endl;
                                float new_cost_p;
                                //cout<<"i is"<<i<<"out of"<<vis_responses.size()<<endl;
                                //cout<<"g is"<<goal<<"out of"<<vis_responses[0].size()<<endl;
                                //cout<<"c is"<<clusterVisitOrder[other]<<"out of"<<vis_responses[0][0].size()<<endl;
                                //cout<<"crit is"<<crit<<"out of"<<vis_responses[0][0][0].size()<<endl;
                                //cout<<0<<" - "<<clusterVisitOrder[0]<<"; "<<1<<" - "<<clusterVisitOrder[1]<<"; "<<2<<" - "<<clusterVisitOrder[2]<<"; "<<endl;
                                if(vis_responses[i][goal][clusterVisitOrder[other]][goalsInClustersRun[pt][other][goal][crit]].cost>=0 ){
                                    //cout<<"Test0"<<endl;
                                    sum+=vis_responses[i][goal][clusterVisitOrder[other]][goalsInClustersRun[pt][other][goal][crit]].costM;
                                    number++;
                                    new_cost_p=vis_responses[i][goal][clusterVisitOrder[other]][goalsInClustersRun[pt][other][goal][crit]].costP;
                                    //cout<<"Test0"<<endl;
                                }
                                else{
                                    //cout<<"Test2"<<endl;
                                    new_cost_p=-1;
                                    //cout<<"Test2"<<endl;
                                    cout<<"Aqui: "<<goal<<" ("<<pt<<"; "<<other<<")"<<endl;

                                }
                                if( costsP[pt][goal]<0 && new_cost_p>=0 ){
                                    //cout<<"Test123"<<endl;
                                    feasible_goals[goal]=true;
                                    newMaxValue[i][goal]=costSensing(LAMBDA,(float)defl[i]);
                                    maxValue[goal]=costSensing(LAMBDA,(float)max(defl[0], defl[1]));
                                    costsP[pt][goal]=new_cost_p;
                                    //cout<<"Test123"<<endl;

                                }
                                else if(new_cost_p>=0 && (costsP[pt][goal]>new_cost_p) ){
                                    //cout<<"Test321"<<endl;
                                    costsP[pt][goal]=new_cost_p;
                                    //cout<<"Test321"<<endl;

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
            for(unsigned int goal=0; goal<goals.size(); goal++){
                if( (costsP[pt][goal]<0) && papositive[i][goal]){
                    if( ( (cluster_centers[pt].i-goals[goal].i)*(cluster_centers[pt].i-goals[goal].i)+(cluster_centers[pt].j-goals[goal].j)*(cluster_centers[pt].j-goals[goal].j) )<=(defl[i]*defl[i]) ){
                        if(raytracing(or_map, cluster_centers[pt].i, cluster_centers[pt].j, goals[goal].i, goals[goal].j, true)){
                            feasible_goals[goal]=true;
                            newMaxValue[i][goal]=costSensing(LAMBDA,(float)defl[i]);
                            maxValue[goal]=costSensing(LAMBDA,(float)max(defl[0], defl[1]));
                            costsP[pt][goal]=costSensingSqDist(LAMBDA,( (cluster_centers[pt].i-goals[goal].i)*(cluster_centers[pt].i-goals[goal].i)+(cluster_centers[pt].j-goals[goal].j)*(cluster_centers[pt].j-goals[goal].j) ));
                        }
                    }
                }
            }
        }
        costsMs[i]=costsM;
        costsPs[i]=costsP;
        clusterPts[i]=cluster_centers;
        points_visited[i]=vector<bool>(cluster_centers.size(), false);
    }

    diff = ros::Time::now() - t0;
    ROS_INFO("Time for Visibility Clusters Determination: %f", diff.toSec());

    /////////////////////////// METHOD 4 ///////////////////////////////////////

    cout<<"----> Method 4 (Vis):"<<endl;
    t0=ros::Time::now();

    paths=vector<vector<int> > (2, vector<int>(1, 0));

    multi_levels=regions.size()*max(max_dist, costSensing(LAMBDA,(float)max(defl[0], defl[1])) ); // ?? think, probably no need for max_dist

    goal_costs=maxValue;

    goal_visited=vector<bool>(goals.size(), false);

    //points_visited[0]=vector<bool>(points_visited[0].size(), false);
    //points_visited[1]=vector<bool>(points_visited[1].size(), false);
    size_points=points_visited[0].size()+points_visited[1].size();

    num_visited=0;
    finished=false;
    while(!finished){
        float n_temp;
        unsigned int robot;

        int depth=3;
        //cout<<goal_visited[0]<<"; "<<newMaxValue[1][0]<<endl;
        FindMax<float, unsigned int> best_point=iterate(costsMs, costsPs, goal_costs, goal_visited, paths, points_visited, newMaxValue, max_dist, multi_levels, robot,n_temp, depth);

        if( (best_point.getVal()<(-max_dist*depth)) || !best_point.valid())
            finished=true;
        else{
            //cout<<"Choose robot "<<robot<<"; cluster point: "<<best_point.getInd()<<endl;
            paths[robot].insert(paths[robot].begin()+(best_point.getP()+1), best_point.getInd()+1);
            for(auto goal=0u; goal<goals.size(); goal++){
                if(maxValue[goal]>=0 && costsPs[robot][best_point.getInd()][goal]>=0){
                    if( costsPs[robot][best_point.getInd()][goal]<= goal_costs[goal] ){
                        goal_costs[goal]=costsPs[robot][best_point.getInd()][goal];
                    }
                    goal_visited[goal]=true;
                }
            }
            points_visited[robot][best_point.getInd()]=true;
        }
        num_visited++;
        if( (num_visited==size_points) )
            finished=true;
    }
    goals_costs=goal_costs;
    goals_visited=goal_visited;

    path_temp=Apath();
    //PointI p_0;
    p_0=pr[0].front();
    pr[0].clear();
    pr[0].push_back(p_0);
    p_0=pr[1].front();
    pr[1].clear();
    pr[1].push_back(p_0);

    cout<<"Robot 0: ";
    for(unsigned int p_i=1;p_i<paths[0].size();p_i++){
        cout<<paths[0][p_i]<<"; ";
        if(paths[0][p_i-1]==0)
            p_0=pr[0].front();
        else
            p_0=clusterPts[0][paths[0][p_i-1]-1];
        //cout<<p_0.i<<":"<<p_0.j<<"; ";
        //cout<<clusterPts[0][paths[0][p_i]-1].i<<":"<<clusterPts[0][paths[0][p_i]-1].j<<"; ";
        path_temp=Astar<float>(p_0, clusterPts[0][paths[0][p_i]-1], msg_rcv[E_MAP+0]);
        pr[0].insert(pr[0].end(), path_temp.points.begin(), path_temp.points.end());
    }
    //if(paths[0].back()==0)
    //    cout<<pr[0].front().i<<":"<<pr[0].front().j<<"; "<<endl;
    //else
    //    cout<<clusterPts[0][paths[0].back()-1].i<<":"<<clusterPts[0][paths[0].back()-1].j<<"; "<<endl;
    cout<<"Robot 1: ";
    for(unsigned int p_i=1;p_i<paths[1].size();p_i++){
        cout<<paths[1][p_i]<<"; ";
        if(paths[1][p_i-1]==0)
            p_0=pr[1].front();
        else
            p_0=clusterPts[1][paths[1][p_i-1]-1];
        path_temp=Astar<float>(p_0, clusterPts[1][paths[1][p_i]-1], msg_rcv[E_MAP+1]);
        pr[1].insert(pr[1].end(), path_temp.points.begin(), path_temp.points.end());
    }
    path_5.poses.clear();
    for(unsigned int p_i=0;p_i<pr[0].size();p_i++){
        p_temp.pose.position=convertI2W(pr[0][p_i], res);
        path_5.poses.push_back(p_temp);
    }
    path_6.poses.clear();
    for(unsigned int p_i=0;p_i<pr[1].size();p_i++){
        p_temp.pose.position=convertI2W(pr[1][p_i], res);
        path_6.poses.push_back(p_temp);
    }
    pub6.publish(path_5);
    pub7.publish(path_6);


    diff = ros::Time::now() - t0;
    ROS_INFO("Time for heuristic planning with Visibity Maps: %f; cost: %f", diff.toSec(), evaluate(costsMs,feasible_goals,paths,goals_costs,goals_visited));

    /////////////////////////// METHOD 5 ///////////////////////////////////////

    cout<<"----> Method 5 (Test Vis Seq):"<<endl;
    t0=ros::Time::now();

    cout<<"Robot 0: ";

    prTest[0].clear();
    prTest[0].push_back(pr[0].front());
    prTest[1].clear();
    prTest[1].push_back(pr[1].front());
    pathTest.assign(2, vector<int>(1, 0));


    //pathTest[0].push_back(1);
    //pathTest[1].push_back(3);
    pathTest[0].push_back(2);
    pathTest[0].push_back(4);
    for(unsigned int robb=0; robb<2; robb++){
        for(unsigned int p_i=1;p_i<pathTest[robb].size();p_i++){
            if(pathTest[robb][p_i-1]==0)
                p_0=pr[robb].front();
            else{
                if((pathTest[robb][p_i-1]-1)<0 || (pathTest[robb][p_i-1]-1)>=(int)clusterPts[robb].size() )
                    break;
                p_0=clusterPts[robb][pathTest[robb][p_i-1]-1];
            }
            if((pathTest[robb][p_i]-1)<0 || (pathTest[robb][p_i]-1)>=(int)clusterPts[robb].size() )
                break;
            path_temp=Astar<float>(p_0, clusterPts[robb][pathTest[robb][p_i]-1], msg_rcv[E_MAP+robb]);
            prTest[robb].insert(prTest[robb].end(), path_temp.points.begin(), path_temp.points.end());
        }
        path_2.poses.clear();
        for(unsigned int p_i=0;p_i<prTest[robb].size();p_i++){
            p_temp.pose.position=convertI2W(prTest[robb][p_i], res);
            path_2.poses.push_back(p_temp);
        }
        if(robb==0)
            pub8.publish(path_2);
        else
            pub9.publish(path_2);
    }

    diff = ros::Time::now() - t0;
    ROS_INFO("Time for planning with Test Sequence: %f; cost: %f", diff.toSec(), evaluate(costsMs,costsPs,feasible_goals,pathTest));


    pl=false;
}

FindMax<float, unsigned int> Multirobotplannersensing::iterate(vector<vector<vector<float> > > costsM, vector<vector<vector<float> > > costsP, vector<float> goal_costs, vector<bool> goal_visited, vector<vector<int> > paths, vector<vector<bool> > points_visited, vector<vector<float> > newMaxValue, float max_dist, float multi_levels, unsigned int& robot_index, float& n,int level){
    FindMax<float, unsigned int> best_point;
    vector<FindMax<float, unsigned int, float> > best_point_temp(2);
    vector<vector<float> > ns(2);
    n=0;
    if(level==0){
        return best_point;
    }
    //cout<<"Level "<<level<<endl;
    for(unsigned int robot=0; robot<paths.size(); robot++){
        //cout<<"Robot "<<robot<<endl;
        for(unsigned int pt=0; pt<points_visited[robot].size();pt++){
            if(!points_visited[robot][pt]){
                float gained_cost=0;
                bool special=false;
                FindMin<float> min_pos;
                for(unsigned int pos=0; pos<(paths[robot].size()-1); pos++){
                    min_pos.iter(costsM[robot][paths[robot][pos]][pt+1]+costsM[robot][pt+1][paths[robot][pos+1]]-costsM[robot][paths[robot][pos]][paths[robot][pos+1]]);
                    //cout<<paths[robot][pos]<<"; "<<pt+1<<"; "<<paths[robot][pos+1]<<"; "<<costsM[robot][paths[robot][pos]][pt+1]<<"; "<<costsM[robot][paths[robot][pos+1]][pt+1]<<"; "<<costsM[robot][paths[robot][pos]][paths[robot][pos+1]]<<"; "<<costsM[robot][paths[robot][pos]][pt+1]+costsM[robot][pt+1][paths[robot][pos+1]]-costsM[robot][paths[robot][pos]][paths[robot][pos+1]]<<endl;
                }
                min_pos.iter(costsM[robot][paths[robot].back()][pt+1]);
                //cout<<costsM[robot][paths[robot].back()][pt+1]<<endl;
                gained_cost+=-min_pos.getVal();
                //float motion_cost=gained_cost;
                //cout<<"Level "<<level<<"; Point "<<pt<<"; motion: "<<min_pos.getVal()<<"; ";
                bool new_goal=false;
                FindMax<float> unfeas_gain;
                for(auto goal=0u; goal<goals.size(); goal++){
                    if(costsP[robot][pt][goal]>=0){
                        if( costsP[robot][pt][goal]<= goal_costs[goal] ){
                            if(!new_goal && !goal_visited[goal]){
                                //cout<<goal<<endl;
                                new_goal=true;
                                //gained_cost+=max_dist;
                                special=true;
                                //cout<<"Special; ";
                            }
                            if(!goal_visited[goal]){
                                //// NewSpecial goals not reachable to other
                                float unfeasible_gain=0;
                                for(unsigned int feas=0; feas<newMaxValue.size(); feas++){
                                    if(newMaxValue[feas][goal]<0)
                                        unfeasible_gain+=multi_levels;
                                }
                                unfeas_gain.iter(unfeasible_gain);
                            }
                            for(auto region=0u; region<goalsRegionsIds[goal].size(); region++){
                                gained_cost+=(goal_costs[goal]-costsP[robot][pt][goal])/goalsInRegions[goalsRegionsIds[goal][region]].size();
                            }
                        }
                    }
                    //else{
                    //    cout<<"Bamos lá ver: "<<goal<<"; CostsP: "<<costsP[robot][pt][goal]<<"; global: "<<goal_costs[goal]<<endl;
                    //}
                }
                //float sensing_cost=gained_cost-motion_cost;
                ////NewSpecial
                if(unfeas_gain.valid())
                    gained_cost+=unfeas_gain.getVal();
                //cout<<"\n(level "<<level<<") Final (" <<pt<<"): "<<gained_cost<<endl;
                //if(level==3)
                //    cout<<"Level "<<level<<"; Robot "<<robot<<"; Point "<<pt<<"; Motion: "<<motion_cost<<"Sens: "<<sensing_cost<<"; Final: "<<gained_cost<<";"<<" Special: "<<(special?"True":"False")<<endl;
                if(gained_cost<0 && !special){
                    best_point_temp[robot].iter(-1-max_dist*level, 0);
                    ns[robot].push_back(0);
                    //cout<<"Level "<<level<<"; Robot "<<robot<<"; Point "<<pt<<"; FinalComplete: "<<"No tree, Stop"<<";"<<endl;
                }
                else{
                    float mid_gained_cost=gained_cost;
                    vector<float> goal_costs_temp=goal_costs;
                    vector<bool> goal_visited_temp=goal_visited;
                    vector<vector<int> > paths_temp=paths;
                    vector<vector<bool> > points_visited_temp=points_visited;
                    paths_temp[robot].insert(paths_temp[robot].begin()+(min_pos.getInd()+1), pt+1);
                    for(auto goal=0u; goal<goals.size(); goal++){
                        if(costsP[robot][pt][goal]>=0){
                            if( costsP[robot][pt][goal]<= goal_costs[goal] ){
                                goal_costs_temp[goal]=costsP[robot][pt][goal];
                            }
                            goal_visited_temp[goal]=true;
                        }
                    }
                    points_visited_temp[robot][pt]=true;
                    float n_temp;
                    unsigned int r_temp;
                    FindMax<float, unsigned int> next=iterate(costsM, costsP, goal_costs_temp, goal_visited_temp, paths_temp, points_visited_temp, newMaxValue, max_dist, multi_levels , r_temp, n_temp, level-1);
                    //cout<<"Next: "<<(next.valid()?next.getVal():-1)<<" ("<<n_temp<<")"<<endl;
                    if(!next.valid()){
                        n_temp=0;
                        gained_cost+=0;
                        //gained_cost+=max_dist*(level-1);
                        ns[robot].push_back(1);
                    }
                    else{
                        //gained_cost=gained_cost/(n_temp+1)+n_temp/(n_temp+1)*next.getVal();
                        gained_cost+=next.getVal();
                        ns[robot].push_back(n_temp+1);
                    }
                    //gained_cost+=next.valid()?next.getVal():0;
                    //cout<<"(level "<<level<<")FinalwNext (" <<pt<<"): "<<gained_cost<<endl;
                    //cout<<"Level "<<level<<"; Robot "<<robot<<"; Point "<<pt<<"; Next: "<<(next.valid()?next.getVal():-1)<<"; FinalComplete: "<<gained_cost<<";"<<endl;
                    best_point_temp[robot].iter(gained_cost, min_pos.getInd(), 0.0, mid_gained_cost);
                }
            }
            else{
                best_point_temp[robot].iter(-1-max_dist*level, 0);
                ns[robot].push_back(0);
            }
        }

    }
    FindMax<float> best_robot;
    for(unsigned int robot=0; robot<paths.size(); robot++){
        if(best_point_temp[robot].valid()){
            best_robot.iter(best_point_temp[robot].getVal());
        }
        else{
            best_robot.iter(-1-max_dist*level);
        }
    }
    if( (best_robot.getVal()<(-max_dist*level)) || !best_robot.valid()){
        //finished=true;
        best_point.clear();
        n=0;
        robot_index=0;
    }
    else{
        robot_index=best_robot.getInd();
        best_point=best_point_temp[robot_index];
        n=ns[robot_index][best_point.getInd()];
    }
    return best_point;
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
//        for (uint32_t i = 0; i < goals.size(); ++i){
//          point.pose.position=convertI2W(goals[i], res);
//          point.id = i;
//          points.markers.push_back(point);
//        }
//        for (uint32_t i = 0; i < bf_responses.size(); ++i){
//            for (uint32_t j = 0; j < bf_responses[i].size(); ++j){
//                for (uint32_t l = 0; l < bf_responses[i][j].size(); ++l){
//                    if(i==0 && l==0){
//                        point.pose.position=convertI2W(bf_responses[i][j][l].perc_pt, res);
//                        point.id = j;
//                        points.markers.push_back(point);
//                    }
//                }
//            }
//        }
//        for (uint32_t i = 0; i < vis_responses.size(); ++i){
//            for (uint32_t j = 0; j < vis_responses[i].size(); ++j){
//                for (uint32_t l = 0; l < vis_responses[i][j].size(); ++l){
//                    for(uint32_t m = 0; m < vis_responses[i][j][l].size(); ++m){
//                        if(i==0 && l==0 && m==0){
//                            point.pose.position=convertI2W(vis_responses[i][j][l][m].perc_pt, res);
//                            point.id = j;
//                            points.markers.push_back(point);
//                        }
//                    }
//                }
//            }
//        }
        for(uint32_t rob=0; rob<2; rob++){
            for (uint32_t i = 0; i < goals.size(); ++i){
                if( !papositive[rob][i]){
                    point.pose.position=convertI2W(goals[i], res);
                    point.id = i;
                    points.markers.push_back(point);
                }
            }
        }
        pub_markers.publish(points);
    }

    if(critsPA[0].size()>0){
        visualization_msgs::MarkerArray points;
        visualization_msgs::Marker point;
        point.header.frame_id = "/map";
        point.header.stamp =  ros::Time::now();
        point.ns = "critsPA0";
        point.action = visualization_msgs::Marker::ADD;
        point.pose.orientation.w = 1.0;
        point.type = visualization_msgs::Marker::SPHERE;
        point.scale.x = res;
        point.scale.y = res;
        point.scale.z = 0.01;
        point.color.r = 1.0;
        point.color.a = 1.0;
        point.lifetime = ros::Duration(2);
        for (unsigned int i = 0; i < critsPA[0].size(); ++i){
            point.pose.position=convertI2W(critsPA[0][i], res);
            point.id = i;
            points.markers.push_back(point);
        }
        pub_markers.publish(points);
    }
    if(critsPA[1].size()>0){
        visualization_msgs::MarkerArray points;
        visualization_msgs::Marker point;
        point.header.frame_id = "/map";
        point.header.stamp =  ros::Time::now();
        point.ns = "critsPA1";
        point.action = visualization_msgs::Marker::ADD;
        point.pose.orientation.w = 1.0;
        point.type = visualization_msgs::Marker::SPHERE;
        point.scale.x = res;
        point.scale.y = res;
        point.scale.z = 0.01;
        point.color.r = 1.0;
        point.color.a = 1.0;
        point.lifetime = ros::Duration(2);
        for (unsigned int i = 0; i < critsPA[1].size(); ++i){
            point.pose.position=convertI2W(critsPA[1][i], res);
            point.id = i;
            points.markers.push_back(point);
        }
        pub_markers.publish(points);
    }
    if(critsVM[0].size()>0){
        visualization_msgs::MarkerArray points;
        visualization_msgs::Marker point;
        point.header.frame_id = "/map";
        point.header.stamp =  ros::Time::now();
        point.ns = "critsVM0";
        point.action = visualization_msgs::Marker::ADD;
        point.pose.orientation.w = 1.0;
        point.type = visualization_msgs::Marker::SPHERE;
        point.scale.x = res;
        point.scale.y = res;
        point.scale.z = 0.01;
        point.color.r = 1.0;
        point.color.a = 1.0;
        point.lifetime = ros::Duration(2);
        for (unsigned int i = 0; i < critsVM[0].size(); ++i){
            point.pose.position=convertI2W(critsVM[0][i], res);
            point.id = i;
            points.markers.push_back(point);
        }
        pub_markers.publish(points);
    }
    if(critsVM[1].size()>0){
        visualization_msgs::MarkerArray points;
        visualization_msgs::Marker point;
        point.header.frame_id = "/map";
        point.header.stamp =  ros::Time::now();
        point.ns = "critsVM1";
        point.action = visualization_msgs::Marker::ADD;
        point.pose.orientation.w = 1.0;
        point.type = visualization_msgs::Marker::SPHERE;
        point.scale.x = res;
        point.scale.y = res;
        point.scale.z = 0.01;
        point.color.r = 1.0;
        point.color.a = 1.0;
        point.lifetime = ros::Duration(2);
        for (unsigned int i = 0; i < critsVM[1].size(); ++i){
            point.pose.position=convertI2W(critsVM[1][i], res);
            point.id = i;
            points.markers.push_back(point);
        }
        pub_markers.publish(points);
    }
    if(crits[0].size()>0){
        visualization_msgs::MarkerArray points;
        visualization_msgs::Marker point;
        point.header.frame_id = "/map";
        point.header.stamp =  ros::Time::now();
        point.ns = "crits0";
        point.action = visualization_msgs::Marker::ADD;
        point.pose.orientation.w = 1.0;
        point.type = visualization_msgs::Marker::SPHERE;
        point.scale.x = res;
        point.scale.y = res;
        point.scale.z = 0.01;
        point.color.r = 1.0;
        point.color.a = 1.0;
        point.lifetime = ros::Duration(2);
        for (unsigned int i = 0; i < crits[0].size(); ++i){
            point.pose.position=convertI2W(crits[0][i], res);
            point.id = i;
            points.markers.push_back(point);
        }
        pub_markers.publish(points);
    }
    if(crits[1].size()>0){
        visualization_msgs::MarkerArray points;
        visualization_msgs::Marker point;
        point.header.frame_id = "/map";
        point.header.stamp =  ros::Time::now();
        point.ns = "crits1";
        point.action = visualization_msgs::Marker::ADD;
        point.pose.orientation.w = 1.0;
        point.type = visualization_msgs::Marker::SPHERE;
        point.scale.x = res;
        point.scale.y = res;
        point.scale.z = 0.01;
        point.color.r = 1.0;
        point.color.a = 1.0;
        point.lifetime = ros::Duration(2);
        for (unsigned int i = 0; i < crits[1].size(); ++i){
            point.pose.position=convertI2W(crits[1][i], res);
            point.id = i;
            points.markers.push_back(point);
        }
        pub_markers.publish(points);
    }


//    tf::StampedTransform transform;
//    try{
//        pos_listener.lookupTransform("/map", "/robot_0/base_link", ros::Time(0), transform);
//    }
//    catch (tf::TransformException ex){
//        return;
//    }
//    float p0x=transform.getOrigin().x();
//    float p0y=transform.getOrigin().y();
//    try{
//        pos_listener.lookupTransform("/map", "/robot_1/base_link", ros::Time(0), transform);
//    }
//    catch (tf::TransformException ex){
//        return;
//    }
//    float p1x=transform.getOrigin().x();
//    float p1y=transform.getOrigin().y();

    visualization_msgs::MarkerArray robots;
    visualization_msgs::Marker robot;
    robot.header.frame_id = "/robot_0/base_link";
    robot.header.stamp =  ros::Time::now();
    robot.ns = "robot0";
    robot.id=0;
    robot.action = visualization_msgs::Marker::ADD;
    robot.pose.orientation.w = 1.0;
    robot.type = visualization_msgs::Marker::SPHERE;
    robot.scale.x = 2.0*((float)infl[0])*res;
    robot.scale.y = 2.0*((float)infl[0])*res;
    robot.scale.z = 0.01;
    robot.color.r = 1.0;
    robot.color.a = 1.0;
    robot.lifetime = ros::Duration(1);
    robots.markers.push_back(robot);
    robot.header.frame_id = "/robot_1/base_link";
    robot.ns="robot1";
    robot.scale.x = 2.0*((float)infl[1])*res;
    robot.scale.y = 2.0*((float)infl[1])*res;
    robot.scale.z = 0.01;
    robot.color.b = 1.0;
    robot.color.r = 0.0;
    robot.color.a = 1.0;
    robot.lifetime = ros::Duration(1);
    robots.markers.push_back(robot);
    pub_rob_markers.publish(robots);
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
    vector<float> temps_sums(regions.size(), 0.0);
    for(unsigned int g=0; g<feasible_goals.size(); g++){
        if(feasible_goals[g] && !goal_visited[g]){
            //for(unsigned int abc=0; abc<feasible_goals.size();abc++){
            //    cout<<abc<<"-"<<feasible_goals[abc]<<":"<<goal_visited[abc]<<"; ";
            //}
            return -1;
        }
        for(auto region=0u; region<goalsRegionsIds[g].size(); region++){
            sum+=(goal_costs[g])/goalsInRegions[goalsRegionsIds[g][region]].size();
            temps_sums[goalsRegionsIds[g][region]]+=(goal_costs[g])/goalsInRegions[goalsRegionsIds[g][region]].size();
            if(stop_th)
                if(sum>th)
                    return sum;
        }
    }
    for(auto it=0u; it<temps_sums.size(); it++){
        cout<<"Target "<<it<<": "<<temps_sums[it]<<endl;
    }
    //float temp=sum;
    //cout<<"Sens: "<<temp<<endl;
    for(unsigned int i=0; i<paths.size(); i++){
        float temp_sum=0;
        for(unsigned int pt=1; pt<paths[i].size(); pt++){
            if( (paths[i][pt-1]<0) || (paths[i][pt-1]>=(int)costM[i].size()) )
                return -1;
            if( (paths[i][pt]<0) || (paths[i][pt]>=(int)costM[i].size()) )
                return -1;
            sum+=costM[i][paths[i][pt]][paths[i][pt-1]];
            temp_sum+=costM[i][paths[i][pt]][paths[i][pt-1]];
            if(stop_th)
                if(sum>th)
                    return sum;
        }
        cout<<"Motion "<<i<<": "<<temp_sum<<endl;
    }
    //cout<<"Motion: "<<sum-temp<<endl;
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
    //cout<<"Testaaaaaaaa"<<endl;
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
    //cout<<"Testeeeeeeeeeee"<<endl;
    return evaluate(costM, feasible_goals, paths, goal_costs, goal_visited, th);
    //cout<<"Testeooooooo"<<endl;
}

float Multirobotplannersensing::costSensing(float K, float dist){
    return K*dist*dist;
}

float Multirobotplannersensing::costSensingSqDist(float K, float dist){
    return K*dist;
}

int main(int argc, char **argv){ 
  ros::init(argc, argv, "multi_sensing_planning");
  ros::NodeHandle nh("~");
  Multirobotplannersensing planner(nh);
  ros::Rate loop_rate(10);

//  vector<int> xxx; xxx.push_back(3); xxx.push_back(1); xxx.push_back(5);
//  Sequence x;
//  cout<<"------ P0 ------"<<endl;
//  x=permutations(xxx,0);
//  x.print();
//  cout<<"------ P1 ------"<<endl;
//  x=permutations(xxx,1);
//  x.print();
//  cout<<"------ P2 ------"<<endl;
//  x=permutations(xxx,2);
//  x.print();

//  cout<<"------ P3 ------"<<endl;
//  x=permutations(xxx,3);
//  x.print();
//  cout<<"------ P4 ------"<<endl;
//  x=permutations(xxx,4);
//  x.print();

//  cout<<"------ C0 ------"<<endl;
//  x=combinations(xxx,0);
//  x.print();
//  cout<<"------ C1 ------"<<endl;
//  x=combinations(xxx,1);
//  x.print();
//  cout<<"------ C2 ------"<<endl;
//  x=combinations(xxx,2);
//  x.print();

//  cout<<"------ C3 ------"<<endl;
//  x=combinations(xxx,3);
//  x.print();
//  cout<<"------ C4 ------"<<endl;
//  x=combinations(xxx,4);
//  x.print();

//  vector<int> gc; gc.push_back(1); gc.push_back(2);
//  vector<int> g1; g1.push_back(11); g1.push_back(12);
//  vector<int> g2; g2.push_back(21); g2.push_back(22);
//  cout<<"------ Comb ------"<<endl;
//  x=combine(g1,g2,gc);
//  x.print();
//  cout<<"------ Perm ------"<<endl;
//  x=permute(g1,g2);
//  x.print();
//  cout<<"------ Test ------"<<endl;
//  x=permute(g1,vector<int>(0));
//  x.print();

  while (ros::ok()){
    ros::spinOnce();
    planner.plan();
    planner.publish();
    loop_rate.sleep();
  }
  return 0;
}
