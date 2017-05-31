#include "ros/ros.h"
#include <vector>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_listener.h"
#include <queue>
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "Astar.hpp"
#include "combinatorics.hpp"

using namespace std;

class Planner{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub1;
    ros::Publisher pub2;
    ros::Publisher pub_markers;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::ServiceServer service;
    ros::ServiceServer service2;
    ros::Subscriber sub_goals;
    tf::TransformListener pos_listener;
    void rcv_map1(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map2(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    bool ask_plan(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
    bool clear(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
    void rcv_goal(const geometry_msgs::PoseStamped::ConstPtr& msg);
    vector<vector<vector<bool> > >  msg_rcv;
    vector<int> count;
    vector<bool> map_rcv;
    bool pl;
    bool getMapValue(int n, int i, int j);
    vector<geometry_msgs::Point> goals;
    int prev_goals;
    float res;
    int width,height;
    PointI convertW2I(geometry_msgs::Point p);
    geometry_msgs::Point convertI2W(PointI p);
    double distance2Path(vector<PointI> path, PointI goal, int r);
    int minDistanceGoal2Path(vector<PointI> path, vector<PointI> goal, int r);
public:
    Planner(ros::NodeHandle nh): nh_(nh){
        pub1 = nh_.advertise<nav_msgs::Path>("path0", 1,true);
        pub2 = nh_.advertise<nav_msgs::Path>("path1", 1,true);
        pub_markers = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1,true);
        sub1 = nh_.subscribe("/robot_0/v_map", 1, &Planner::rcv_map1, this);
        sub2 = nh_.subscribe("/robot_1/v_map", 1, &Planner::rcv_map2, this);
        service = nh_.advertiseService("plan", &Planner::ask_plan, this);
        service2 = nh_.advertiseService("clear", &Planner::clear, this);
        sub_goals = nh_.subscribe("/move_base_simple/goal", 1, &Planner::rcv_goal, this);
        count.assign(2,0);
        map_rcv.assign(2,false);
        msg_rcv.resize(2);
        pl=false;
        goals.clear();
        prev_goals=goals.size();
    }
    ~Planner(){
    }
    void plan(void);
    void publish(void);
};

void Planner::rcv_map1(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    msg_rcv[0].assign(msg->info.width, vector<bool>(msg->info.height,false));
    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
            if(*mapDataIterC == 0){
                msg_rcv[0][j][i]=true;
            }
            else{
                msg_rcv[0][j][i]=false;
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
    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
            if(*mapDataIterC == 0){
                msg_rcv[1][j][i]=true;
            }
            else{
                msg_rcv[1][j][i]=false;
            }
            mapDataIterC++;
        }
    }
    map_rcv[1]=true;
    count[1]=count[1]+1;
}

bool Planner::getMapValue(int n, int i, int j){
    return msg_rcv[n][i][j];
}

bool Planner::ask_plan(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res){
    pl=true;
    return true;
}

bool Planner::clear(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res){
    pl=false;
    goals.clear();
    nav_msgs::Path path_0,path_1;
    path_0.poses.clear();
    path_0.header.frame_id = "/map";
    path_0.header.stamp =  ros::Time::now();
    path_0.poses.clear();
    path_1=path_0;
    pub1.publish(path_0);
    pub2.publish(path_1);
    return true;
}

void Planner::rcv_goal(const geometry_msgs::PoseStamped::ConstPtr& msg){
    goals.push_back(msg->pose.position);
}

PointI Planner::convertW2I(geometry_msgs::Point p){
    PointI pf(p.x/res,p.y/res);
    return pf;
}

geometry_msgs::Point Planner::convertI2W(PointI p){
    geometry_msgs::Point pf;
    pf.x=p.i*res;
    pf.y=p.j*res;
    return pf;
}

double Planner::distance2Path(vector<PointI> path, PointI goal, int r){
    vector<double> heuristic(0);
    vector<int> pos(0);
    for(unsigned int p=0;p<path.size();p++){
        if (!msg_rcv[r][path[p].i][path[p].j])
            continue;
        double heur=(double) (path[p].i-goal.i)*(path[p].i-goal.i)+(path[p].j-goal.j)*(path[p].j-goal.j);
        vector<double>::iterator heurIt = heuristic.begin();
        vector<int>::iterator posIt = pos.begin();
        while(heurIt!=heuristic.end()){
            if(heur<*heurIt)
                break;
            heurIt++;
            posIt++;
        }
        heuristic.insert(heurIt,heur);
        pos.insert(posIt,p);
    }
    double min_real=0;
    Apath pathT;
    for(unsigned int h=0;h<(heuristic.size());h++){
        pathT=Astar<float>(path[pos[h]], goal, msg_rcv[r], true);
        if(h==0)
            min_real=pathT.cost;
        else{
            if(pathT.cost<min_real)
                min_real=pathT.cost;
            if(h<(heuristic.size()-1)){
                if( (min_real*min_real)<heuristic[h+1])
                    return min_real;
                else
                    continue;
            }
        }
    }
    return pathT.cost;
}

int Planner::minDistanceGoal2Path(vector<PointI> path, vector<PointI> goal, int r){
    vector<double> heuristic(0);
    vector<int> pos(0);
    vector<int> gp(0);
    for(unsigned int g=0;g<goal.size();g++){
        for( unsigned int p=0;p<path.size();p++){
            if (!msg_rcv[r][path[p].i][path[p].j])
                continue;
            double heur=(double) (path[p].i-goal[g].i)*(path[p].i-goal[g].i)+(path[p].j-goal[g].j)*(path[p].j-goal[g].j);
            vector<double>::iterator heurIt = heuristic.begin();
            vector<int>::iterator posIt = pos.begin();
            vector<int>::iterator gpIt = gp.begin();
            while(heurIt!=heuristic.end()){
                if(heur<*heurIt)
                    break;
                heurIt++;
                posIt++;
                gpIt++;
            }
            heuristic.insert(heurIt,heur);
            pos.insert(posIt,p);
            gp.insert(gpIt,g);
        }
    }
    double min_real=0;
    int g_real=0;
    Apath pathT;
    for(unsigned int h=0;h<(heuristic.size());h++){
        pathT=Astar<float>(path[pos[h]], goal[gp[h]], msg_rcv[r], true);
        if(h==0){
            min_real=pathT.cost;
            g_real=gp[h];
        }
        else{
            if(pathT.cost<min_real){
                min_real=pathT.cost;
                g_real=gp[h];
            }
            if(h<(heuristic.size()-1)){
                if( (min_real*min_real)<heuristic[h+1])
                    return g_real;
                else
                    continue;
            }
        }
    }
    return g_real;
}

double cost_function(double a, double b){
    return max(a,b);
}

void Planner::plan(void){
    if(map_rcv[0] && map_rcv[1] && pl && (goals.size()>0) ){
        nav_msgs::Path path_0,path_1;
        vector<vector<PointI> > pr(2,vector<PointI>(0));
        vector< vector<int> > pg(2,vector<int>(0)), gr(4,vector<int>(0));
        vector<PointI> g;
        vector<double> pc(2,0.0);
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
        PointI pi_temp( (int) round((transform.getOrigin().x())/res), (int) round((transform.getOrigin().y())/res) );
        p_temp.pose.position=convertI2W(pi_temp);
        path_0.poses.push_back(p_temp);
        pr[0].push_back(pi_temp);
        g.push_back(pr[0].back());
        try{
            pos_listener.lookupTransform("/map", "/robot_1/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_INFO("%s",ex.what());
          return ;
        }
        pi_temp=PointI( (int) round((transform.getOrigin().x())/res), (int) round((transform.getOrigin().y())/res) );
        p_temp.pose.position=convertI2W(pi_temp);
        path_1.poses.push_back(p_temp);
        pr[1].push_back(pi_temp);
        g.push_back(pr[1].back());
        for(unsigned int i=0;i<goals.size();i++){
            p_temp.pose.position=goals[i];
            pi_temp=convertW2I(p_temp.pose.position);
            g.push_back(pi_temp);
            if( msg_rcv[0][pi_temp.i][pi_temp.j] && !msg_rcv[1][pi_temp.i][pi_temp.j] )
                gr[0].push_back(2+i);
            else if ( msg_rcv[1][pi_temp.i][pi_temp.j] && !msg_rcv[0][pi_temp.i][pi_temp.j] )
                gr[1].push_back(2+i);
            else if ( msg_rcv[1][pi_temp.i][pi_temp.j] && msg_rcv[0][pi_temp.i][pi_temp.j] )
                gr[2].push_back(2+i);
            else
                gr[3].push_back(2+i);
        }
        vector<bool> gt(g.size(),false);
        int g0s=gr[0].size(), g1s=gr[1].size(),gbs=gr[2].size();
        vector<vector<vector<Apath> > > mat(g.size(),vector<vector<Apath> >(g.size(),vector<Apath>(2,Apath())));
        Apath path;
        pg[0].push_back(0);
        pg[1].push_back(1);

        /////////////////////////// METHOD 1 ///////////////////////////////////////

        ros::Time t01=ros::Time::now();
        cout<<"----> Method 1:"<<endl;
        if(g0s>0){
            int max_cost=gr[0][0];
            for(unsigned int i=0;i<gr[0].size();i++){
                path=Astar<float>(pr[0].front(), g[gr[0][i]], msg_rcv[0], true);
                mat[0][gr[0][i]][0]=path;
                if(path.cost>mat[0][max_cost][0].cost)
                    max_cost=gr[0][i];
            }
            pg[0].push_back(max_cost);
            gt[max_cost]=true;
            pc[0]+=mat[0][max_cost][0].cost;
        }
        if(g1s>0){
            int max_cost=gr[1][0];
            for(unsigned int i=0;i<gr[1].size();i++){
                path=Astar<float>(pr[1].front(), g[gr[1][i]], msg_rcv[1], true);
                mat[1][gr[1][i]][1]=path;
                if(path.cost>mat[1][max_cost][1].cost)
                    max_cost=gr[1][i];
            }
            pg[1].push_back(max_cost);
            gt[max_cost]=true;
            pc[1]+=mat[1][max_cost][1].cost;
        }
        if(g0s>0)
        {
            while(pg[0].size()<(gr[0].size()+1) )
            {
                int m_cost;
                double m_c=0;
                double cost;
                unsigned int m_i=0;
                bool first=true;
                for(unsigned int i=0;i<gr[0].size();i++)
                {
                    if(!gt[gr[0][i]])
                    {
                        double min_c=0;
                        unsigned int min_cost=0;
                        for(unsigned int j=1;j<pg[0].size();j++){
                            if (mat[gr[0][i]][pg[0][j]][0].cost==-1){
                                path=Astar<float>(g[gr[0][i]], g[pg[0][j]], msg_rcv[0], true);
                                mat[gr[0][i]][pg[0][j]][0]=path;
                            }
                            if (mat[pg[0][j-1]][gr[0][i]][0].cost==-1){
                                path=Astar<float>(g[pg[0][j-1]], g[gr[0][i]], msg_rcv[0], true);
                                mat[pg[0][j-1]][gr[0][i]][0]=path;
                            }
                            cost=(mat[pg[0][j-1]][gr[0][i]][0].cost+mat[gr[0][i]][pg[0][j]][0].cost-mat[pg[0][j-1]][pg[0][j]][0].cost);
                            if(j==1){
                                min_c=cost;
                                min_cost=j;
                            }
                            else
                                if(cost<min_c){
                                    min_c=cost;
                                    min_cost=j;
                                }
                        }
                        int j=pg[0].size()-1;
                        if (mat[pg[0][j]][gr[0][i]][0].cost==-1){
                            path=Astar<float>(g[pg[0][j]], g[gr[0][i]], msg_rcv[0], true);
                            mat[pg[0][j]][gr[0][i]][0]=path;
                        }
                        cost= mat[pg[0][j]][gr[0][i]][0].cost;
                        if(cost<min_c){
                            min_cost=j+1;
                            min_c=cost;
                        }
                        if(first){
                            m_c=min_c;
                            m_cost=min_cost;
                            m_i=i;
                            first=false;
                        }
                        else
                            if(min_c<m_c){
                                m_c=min_c;
                                m_cost=min_cost;
                                m_i=i;
                            }
                    }
                }
                vector<int>::iterator it = pg[0].begin();
                pg[0].insert(it+m_cost,gr[0][m_i]);
                gt[gr[0][m_i]]=true;
                pc[0]+=m_c;
            }
        }
        if(g1s>0)
        {
            while(pg[1].size()<(gr[1].size()+1) )
            {
                int m_cost;
                double m_c=0;
                double cost;
                unsigned int m_i=0;
                bool first=true;
                for(unsigned int i=0;i<gr[1].size();i++)
                {
                    if(!gt[gr[1][i]])
                    {
                        double min_c=0;
                        unsigned int min_cost=0;
                        for(unsigned int j=1;j<pg[1].size();j++){
                            if (mat[gr[1][i]][pg[1][j]][1].cost==-1){
                                path=Astar<float>(g[gr[1][i]],g[pg[1][j]], msg_rcv[1], true);
                                mat[gr[1][i]][pg[1][j]][1]=path;
                            }
                            if (mat[pg[1][j-1]][gr[1][i]][1].cost==-1){
                                path=Astar<float>(g[pg[1][j-1]], g[gr[1][i]], msg_rcv[1], true);
                                mat[pg[1][j-1]][gr[1][i]][1]=path;
                            }
                            cost=(mat[pg[1][j-1]][gr[1][i]][1].cost+mat[gr[1][i]][pg[1][j]][1].cost-mat[pg[1][j-1]][pg[1][j]][1].cost);
                            if(j==1){
                                min_c=cost;
                                min_cost=j;
                            }
                            else
                                if(cost<min_c)
                                {
                                    min_c=cost;
                                    min_cost=j;
                                }
                        }
                        int j=pg[1].size()-1;
                        if (mat[pg[1][j]][gr[1][i]][1].cost==-1){
                            path=Astar<float>(g[pg[1][j]], g[gr[1][i]], msg_rcv[1], true);
                            mat[pg[1][j]][gr[1][i]][1]=path;
                        }
                        cost= mat[pg[1][j]][gr[1][i]][1].cost;
                        if(cost<min_c){
                            min_cost=j+1;
                            min_c=cost;
                        }
                        if(first){
                            m_c=min_c;
                            m_cost=min_cost;
                            m_i=i;
                            first=false;
                        }
                        else
                            if(min_c<m_c)
                            {
                                m_c=min_c;
                                m_cost=min_cost;
                                m_i=i;
                            }
                    }
                }
                vector<int>::iterator it = pg[1].begin();
                pg[1].insert(it+m_cost,gr[1][m_i]);
                gt[gr[1][m_i]]=true;
                pc[1]+=m_c;
            }
        }
        if(gbs>0)
        {
            for(int r=0;r<2;r++)
            {
                if (pg[r].size()>1)
                    continue;
                int r_o=(r+1)%2;
                int max_cost;
                double max_c=0;
                for(unsigned int i=0;i<gr[2].size();i++)
                {
                    if(gt[gr[2][i]])
                        continue;
                    double cost=0;
                    if (pg[r_o].size()==1){
                        path=Astar<float>(g[pg[r_o][0]], g[gr[2][i]], msg_rcv[r_o], true);
                        mat[pg[r_o][0]][gr[2][i]][r_o]=path;
                        cost=path.cost;
                    }
                    else{
                        double min_c=0;
                        for(unsigned int j=1;j<pg[r_o].size();j++){
                            if (mat[gr[2][i]][pg[r_o][j]][r_o].cost==-1){
                                path=Astar<float>(g[gr[2][i]],g[pg[r_o][j]], msg_rcv[r_o], true);
                                mat[gr[2][i]][pg[r_o][j]][r_o]=path;
                            }
                            if (mat[pg[r_o][j-1]][gr[2][i]][r_o].cost==-1){
                                path=Astar<float>(g[pg[r_o][j-1]], g[gr[2][i]], msg_rcv[r_o], true);
                                mat[pg[r_o][j-1]][gr[2][i]][r_o]=path;
                            }
                            cost=(mat[pg[r_o][j-1]][gr[2][i]][r_o].cost+mat[gr[2][i]][pg[r_o][j]][r_o].cost-mat[pg[r_o][j-1]][pg[r_o][j]][r_o].cost);
                            if(j==1){
                                min_c=cost;
                            }
                            else
                                if(cost<min_c)
                                {
                                    min_c=cost;
                                }
                        }
                        int j=pg[r_o].size()-1;
                        if (mat[pg[r_o][j]][gr[2][i]][r_o].cost==-1){
                            path=Astar<float>(g[pg[r_o][j]], g[gr[2][i]], msg_rcv[r_o], true);
                            mat[pg[r_o][j]][gr[2][i]][r_o]=path;
                        }
                        cost= mat[pg[r_o][j]][gr[2][i]][r_o].cost;
                        if(cost<min_c){
                            min_c=cost;
                        }
                        cost=min_c;
                    }
                    path=Astar<float>(g[pg[r].front()], g[gr[2][i]], msg_rcv[r], true);
                    mat[pg[r].front()][gr[2][i]][r]=path;
                    cost+=path.cost;
                    if(i==0){
                        max_c=cost;
                        max_cost=gr[2][i];
                    }
                    else
                        if(cost>max_c){
                            max_c=cost;
                            max_cost=gr[2][i];
                        }
                }
                pg[r].push_back(max_cost);
                gt[max_cost]=true;
                pc[r]+=mat[r][max_cost][r].cost;
            }
        }
        while( (pg[0].size()+pg[1].size())<g.size() )
        {
            unsigned int m_cost0,m_cost1;
            double m_c0,m_c1;
            double cost,cost0,cost1;
            double m_i0=0,m_i1=0,c0=0,c1=0;
            bool first=true;
            for(unsigned int i=0;i<gr[2].size();i++)
            {
                if(!gt[gr[2][i]])
                {
                    double min_c0=0,min_c1=0;
                    unsigned int min_cost0,min_cost1;
                    for(unsigned int j=1;j<pg[0].size();j++){
                        if (mat[gr[2][i]][pg[0][j]][0].cost==-1){
                            path=Astar<float>(g[gr[2][i]], g[pg[0][j]], msg_rcv[0], true);
                            mat[gr[2][i]][pg[0][j]][0]=path;
                        }
                        if (mat[pg[0][j-1]][gr[2][i]][0].cost==-1){
                            path=Astar<float>(g[pg[0][j-1]], g[gr[2][i]], msg_rcv[0], true);
                            mat[pg[0][j-1]][gr[2][i]][0]=path;
                        }
                        cost=(mat[pg[0][j-1]][gr[2][i]][0].cost+mat[gr[2][i]][pg[0][j]][0].cost-mat[pg[0][j-1]][pg[0][j]][0].cost);
                        if(j==1){
                            min_c0=cost;
                            min_cost0=j;
                        }
                        else
                            if(cost<min_c0)
                            {
                                min_c0=cost;
                                min_cost0=j;
                            }
                    }
                    unsigned int j=pg[0].size()-1;
                    if (mat[pg[0][j]][gr[2][i]][0].cost==-1){
                        path=Astar<float>(g[pg[0][j]], g[gr[2][i]], msg_rcv[0], true);
                        mat[pg[0][j]][gr[2][i]][0]=path;
                    }
                    cost= mat[pg[0][j]][gr[2][i]][0].cost;
                    if(cost<min_c0){
                        min_cost0=j+1;
                        min_c0=cost;
                    }
                    for(unsigned int j=1;j<pg[1].size();j++){
                        if (mat[gr[2][i]][pg[1][j]][1].cost==-1){
                            path=Astar<float>(g[gr[2][i]],g[pg[1][j]], msg_rcv[1], true);
                            mat[gr[2][i]][pg[1][j]][1]=path;
                        }
                        if (mat[pg[1][j-1]][gr[2][i]][1].cost==-1){
                            path=Astar<float>(g[pg[1][j-1]], g[gr[2][i]], msg_rcv[1], true);
                            mat[pg[1][j-1]][gr[2][i]][1]=path;
                        }
                        cost=(mat[pg[1][j-1]][gr[2][i]][1].cost+mat[gr[2][i]][pg[1][j]][1].cost-mat[pg[1][j-1]][pg[1][j]][1].cost);
                        if(j==1){
                            min_c1=cost;
                            min_cost1=j;
                        }
                        else
                            if(cost<min_c1)
                            {
                                min_c1=cost;
                                min_cost1=j;
                            }
                    }
                    j=pg[1].size()-1;
                    if (mat[pg[1][j]][gr[2][i]][1].cost==-1){
                        path=Astar<float>(g[pg[1][j]], g[gr[2][i]], msg_rcv[1], true);
                        mat[pg[1][j]][gr[2][i]][1]=path;
                    }
                    cost= mat[pg[1][j]][gr[2][i]][1].cost;
                    if(cost<min_c1){
                        min_cost1=j+1;
                        min_c1=cost;
                    }
                    cost0=min_c0-min_c1;
                    cost1=min_c1-min_c0;
                    if(first){
                        m_c0=cost0;
                        m_cost0=min_cost0;
                        m_i0=i;
                        c0=min_c0;
                        m_c1=cost1;
                        m_cost1=min_cost1;
                        m_i1=i;
                        c1=min_c1;
                        first=false;
                    }
                    else{
                        if(cost1<m_c1){
                            m_c1=cost1;
                            m_cost1=min_cost1;
                            m_i1=i;
                            c1=min_c1;
                        }
                        if(cost0<m_c0){
                            m_c0=cost0;
                            m_cost0=min_cost0;
                            m_i0=i;
                            c0=min_c0;
                        }
                    }

                }
            }
            cout<<"Robot 0 cost: "<<(pc[0]) <<endl;
            cout<<"Robot 0 future cost: "<<(pc[0]+c0) <<endl;
            cout<<"Robot 1 cost: "<<(pc[1]) <<endl;
            cout<<"Robot 1 future cost: "<<(pc[1]+c1) <<endl;
            if( (pc[0]+c0)<(pc[1]+c1) ){
                cout<<"Robot 0 added to path"<<endl;
                vector<int>::iterator it = pg[0].begin();
                pg[0].insert(it+m_cost0,gr[2][m_i0]);
                gt[gr[2][m_i0]]=true;
                pc[0]+=c0;
            }
            else{
                cout<<"Robot 1 added to path"<<endl;
                vector<int>::iterator it = pg[1].begin();
                pg[1].insert(it+m_cost1,gr[2][m_i1]);
                gt[gr[2][m_i1]]=true;
                pc[1]+=c1;
            }
        }
        cout<<"Robo 0: ";
        for(unsigned int i=0;i<pg[0].size();i++){
            cout<<pg[0][i]<<" ";
        }
        cout<<endl;
        cout<<"Robo 1: ";
        for(unsigned int i=0;i<pg[1].size();i++){
            cout<<pg[1][i]<<" ";
        }
        cout<<endl;
        cout<<"Goals: ";
        for(unsigned int i=0;i<gt.size();i++){
            cout<<gt[i]<<" ";
        }
        cout<<endl;
        cout<<"Robot 0 cost: "<<(pc[0]) <<endl;
        cout<<"Robot 1 cost: "<<(pc[1]) <<endl;
        for(unsigned int p_i=1;p_i<pg[0].size();p_i++){
            pr[0].insert(pr[0].end(), mat[pg[0][p_i-1]][pg[0][p_i]][0].points.begin(), mat[pg[0][p_i-1]][pg[0][p_i]][0].points.end());
        }
        for(unsigned int p_i=1;p_i<pg[1].size();p_i++){
            pr[1].insert(pr[1].end(), mat[pg[1][p_i-1]][pg[1][p_i]][1].points.begin(), mat[pg[1][p_i-1]][pg[1][p_i]][1].points.end());
        }
        path_0.poses.clear();
        for(unsigned int p_i=0;p_i<pr[0].size();p_i++){
            p_temp.pose.position=convertI2W(pr[0][p_i]);
            path_0.poses.push_back(p_temp);
        }
        path_1.poses.clear();
        for(unsigned int p_i=0;p_i<pr[1].size();p_i++){
            p_temp.pose.position=convertI2W(pr[1][p_i]);
            path_1.poses.push_back(p_temp);
        }
        pub1.publish(path_0);
        pub2.publish(path_1);
        pl=false;
        ros::Duration diff = ros::Time::now() - t01;
        cout<<diff<<endl;

        /////////////////////////// METHOD 2 ///////////////////////////////////////

        cout<<"----> Method 2:"<<endl;
        ros::Time t02=ros::Time::now();
        Sequence perm=combine(gr[0],gr[1],gr[2]);
        vector<int> goal_opt1, goal_opt2;
        double cost_1max, cost_2max, cost_max=0;
        vector<vector<vector<double> > > mat_c(g.size(),vector<vector<double> >(g.size(),vector<double>(2,-1)));
        int counter_opt=0;
        for(unsigned int i=0;i<perm.seq.size();i++)
        {
            double cost=0;
            double cost2=0;
            for(unsigned int j=0;j<perm.seq[i].size();j++){
                if(j==0){
                    if(mat_c[pg[0][0]][perm.seq[i][j]][0]==-1){
                        path=Astar<float>(g[pg[0][0]], g[perm.seq[i][j]], msg_rcv[0], true);
                        mat_c[pg[0][0]][perm.seq[i][j]][0]=path.cost;
                    }
                    if(mat_c[pg[0][0]][perm.seq[i][j]][0]<0){
                        cost=-2; break;
                    }
                    cost+=mat_c[pg[0][0]][perm.seq[i][j]][0];
                }
                else{
                    if(mat_c[perm.seq[i][j-1]][perm.seq[i][j]][0]==-1){
                        path=Astar<float>(g[perm.seq[i][j-1]], g[perm.seq[i][j]], msg_rcv[0], true);
                        mat_c[perm.seq[i][j-1]][perm.seq[i][j]][0]=path.cost;
                    }
                    if(mat_c[perm.seq[i][j-1]][perm.seq[i][j]][0]<0){
                        cost=-2; break;
                    }
                    cost+=mat_c[perm.seq[i][j-1]][perm.seq[i][j]][0];
                }
                if( cost_function(cost,cost2)>cost_max && i>0){
                    cost=-2; counter_opt++; break;
                }
            }
            if(cost==-2)
                continue;
            for(unsigned int j=0;j<perm.rem[i].size();j++){
                if(j==0){
                    if(mat_c[pg[1][0]][perm.rem[i][j]][1]==-1){
                        path=Astar<float>(g[pg[1][0]], g[perm.rem[i][j]], msg_rcv[1], true);
                        mat_c[pg[1][0]][perm.rem[i][j]][1]=path.cost;
                    }
                    if(mat_c[pg[1][0]][perm.rem[i][j]][1]<0){
                        cost2=-2; break;
                    }
                    cost2+=mat_c[pg[1][0]][perm.rem[i][j]][1];
                }
                else{
                    if(mat_c[perm.rem[i][j-1]][perm.rem[i][j]][1]==-1){
                        path=Astar<float>(g[perm.rem[i][j-1]], g[perm.rem[i][j]], msg_rcv[1], true);
                        mat_c[perm.rem[i][j-1]][perm.rem[i][j]][1]=path.cost;
                    }
                    if(mat_c[perm.rem[i][j-1]][perm.rem[i][j]][1]<0){
                        cost2=-2; break;
                    }
                    cost2+=mat_c[perm.rem[i][j-1]][perm.rem[i][j]][1];
                }
                if( cost_function(cost,cost2)>cost_max && i>0 ){
                    cost2=-2; counter_opt++; break;
                }
            }
            if(cost2==-2)
                continue;
            if(i==0){
                cost_1max=cost;
                cost_2max=cost2;
                cost_max=cost_function(cost_1max,cost_2max);
                goal_opt1=perm.seq[i];
                goal_opt2=perm.rem[i];
            }
            else{
                if( cost_function(cost,cost2)<cost_max ){
                    cost_1max=cost;
                    cost_2max=cost2;
                    cost_max=cost_function(cost_1max,cost_2max);
                    goal_opt1=perm.seq[i];
                    goal_opt2=perm.rem[i];
                }
            }
        }
        cout<<"Robo 0: 0 ";
        for(unsigned int i=0;i<goal_opt1.size();i++){
            cout<<goal_opt1[i]<<" ";
        }
        cout<<endl;
        cout<<"Robo 1: 1 ";
        for(unsigned int i=0;i<goal_opt2.size();i++){
            cout<<goal_opt2[i]<<" ";
        }
        cout<<endl;
        cout<<"Robot 0 cost: "<<cost_1max <<endl;
        cout<<"Robot 1 cost: "<<cost_2max <<endl;
        diff = ros::Time::now() - t02;
        cout<<"Number of optimizations: "<<counter_opt<<endl;
        cout<<diff<<endl;

        cout<<"----> Testing time impossible A*"<<endl;

        ros::Time t03=ros::Time::now();
        int counter=0;
        for(unsigned int i=0;i<gr[0].size();i++){
           path=Astar<float>(g[pg[1][0]], g[gr[0][i]], msg_rcv[1], true);
           if(path.cost==-2)
               counter++;
        }
        cout<<counter<<"/"<<gr[0].size()<<endl;
        counter=0;
        for(unsigned int i=0;i<gr[1].size();i++){
           path=Astar<float>(g[pg[0][0]], g[gr[1][i]], msg_rcv[0], true);
           if(path.cost==-2)
               counter++;
        }
        cout<<counter<<"/"<<gr[1].size()<<endl;
        counter=0;
        for(unsigned int i=0;i<gr[3].size();i++){
           path=Astar<float>(g[pg[0][0]], g[gr[3][i]], msg_rcv[0], true);
           if(path.cost==-2)
               counter++;
           path=Astar<float>(g[pg[1][0]], g[gr[3][i]], msg_rcv[1], true);
           if(path.cost==-2)
               counter++;
        }
        cout<<counter<<"/"<<2*gr[3].size()<<endl;
        diff = ros::Time::now() - t03;
        cout<<"Time to fail all impossible goals with A*:"<<diff<<endl;
    }
}

void Planner::publish(void){
    if(goals.size()>0){
        visualization_msgs::MarkerArray points;
        visualization_msgs::Marker point;
        point.header.frame_id = "/map";
        point.header.stamp =  ros::Time::now();
        point.ns = "points_and_lines";
        point.action = visualization_msgs::Marker::ADD;
        point.pose.orientation.w = 1.0;
        point.type = visualization_msgs::Marker::SPHERE;
        point.scale.x = 0.2;
        point.scale.y = 0.2;
        point.scale.z = 0.2;
        point.color.g = 1.0f;
        point.color.a = 1.0;
        point.lifetime = ros::Duration(2);
        for (uint32_t i = 0; i < goals.size(); ++i){
          point.pose.position=goals[i];
          point.id = i;
          points.markers.push_back(point);
        }
        pub_markers.publish(points);
        prev_goals=goals.size();
    }
}

int main(int argc, char **argv){ 
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh("~");
    Planner planner(nh);
    ros::Rate loop_rate(10);
    while (ros::ok()){
        ros::spinOnce();
        planner.plan();
        planner.publish();
        loop_rate.sleep();
    }
    return 0;
}
