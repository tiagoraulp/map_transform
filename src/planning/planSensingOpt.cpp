#include "ros/ros.h"
#include <vector>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_listener.h"
#include <map_transform/VisCom.h>
#include <map_transform/VisNode.h>
#include <map_transform/PAstar.h>

#include <queue>
#include "std_msgs/String.h"

#include "std_srvs/Empty.h"

#include "ray.hpp"
#include "vector_utils.hpp"

#include <fstream>

using namespace std;

const int dir=8; // number of possible directions to go at any position
//if dir==4
//static int dx[dir]={1, 0, -1, 0};
//static int dy[dir]={0, 1, 0, -1};
//if dir==8
static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};

class PointI{

public:
    int i;
    int j;
    PointI(int a, int b)
    {
        i=a;
        j=b;
    }
    PointI()
    {
        i=0;
        j=0;
    }


};

class Apath{
public:
    vector<PointI> points;
    float cost;

    Apath()
    {
        cost=-1;
        points.clear();
    }
};

float k1=1;

float k2=0.04;

bool quad=true;

//const float k2=1;

//bool quad=false;


template<typename T>
class node
{
    int infl, defl;
    // current position
    int xPos;
    int yPos;
    // total distance already travelled to reach the node
    T level;
    // priority=level+remaining distance estimate
    T priority;  // smaller: higher priority
    T sens;
    float opt;
    map_transform::VisNode crit;
    float K;
    bool bfs;

    int Gx;
    int Gy;

    float (node::*costEst) (const int x, const int y) const;
    float (node::*costSens) (const int x, const int y) const;

    bool power;

    float costSensing(const int x,const int y) const
    {
        //return sqrt(x*x+y*y);
        if( ( (x)*(x)+(y)*(y) )>(defl*defl) )
            return -2;
        else
            return k2*sqrt(x*x+y*y);
    }

    float costSensing2(const int x,const int y) const
    {
        //return sqrt(x*x+y*y);
        if( ( (x)*(x)+(y)*(y) )>(defl*defl) )
            return -2;
        else
            return k2*(x*x+y*y);
    }


    inline float costEstimateDist(const float d, const int ss) const
    {
        return d/14.14213562*14;
    }

    inline float costEstimateDist(const float d, const float ss) const
    {
        return d;
    }

    float costEstimate(const int x,const int y) const
    {
        //return sqrt(x*x+y*y);
        if(k1>k2)
        {
            if( ( (x)*(x)+(y)*(y) )>(defl*defl) )
                return k1*costEstimateDist((sqrt(x*x+y*y)-defl), sens)+k2*defl;
            else
                return k2*sqrt(x*x+y*y);
        }
        else
        {
            if(opt<0)
                return k1*costEstimateDist(sqrt(x*x+y*y),sens);
            else
            {
                float est1;
                if( ( (x)*(x)+(y)*(y) )>=(opt*opt) )
                    est1=k1*costEstimateDist((sqrt(x*x+y*y)-opt),sens)+k2*opt;
                else
                    //return k2*sqrt(x*x+y*y);
                    est1=k2*opt;

                float est2;
                FindMin<float> min_est;
                for(int i=0;i<crit.points.size();i++)
                {
                    float xx=crit.points[i].position.x-xPos;
                    float yy=crit.points[i].position.y-yPos;

                    float xG=crit.points[i].position.x-Gx;
                    float yG=crit.points[i].position.y-Gy;

                    min_est.iter(k1*(sqrt(xx*xx+yy*yy)-2*infl)+k2*sqrt(xG*xG+yG*yG));
                }
                est2=min_est.getVal();

                return max(est1,est2);
                //return est1;
            }
        }
    }

    float costEstimate2(const int x,const int y) const
    {
        //return sqrt(x*x+y*y);
        if(opt<0)
        {
            if(K<defl)
                if( ( (x)*(x)+(y)*(y) )>=(K*K) )
                    return k1*costEstimateDist((sqrt(x*x+y*y)-K),sens)+k2*K*K;
                else
                    return k2*(x*x+y*y);
            else
                if( ( (x)*(x)+(y)*(y) )>=(defl*defl) )
                    return k1*costEstimateDist((sqrt(x*x+y*y)-defl),sens)+k2*defl*defl;
                else
                    return k2*(x*x+y*y);
        }
        else
        {
            float est1;

            if(K<defl)
                if( K>opt )
                    if( ( (x)*(x)+(y)*(y) )>=(K*K) )
                        est1= k1*costEstimateDist((sqrt(x*x+y*y)-K),sens)+k2*K*K;
                    else
                        if( ( (x)*(x)+(y)*(y) )>=(opt*opt) )
                            est1= k2*(x*x+y*y);
                        else
                            est1= k2*opt*opt;
                else
                    if( ( (x)*(x)+(y)*(y) )>=(opt*opt) )
                        est1= k1*costEstimateDist((sqrt(x*x+y*y)-opt),sens)+k2*opt*opt;
                    else
                        est1= k2*opt*opt;
            else
                if( ( (x)*(x)+(y)*(y) )>=(defl*defl) )
                    est1= k1*costEstimateDist((sqrt(x*x+y*y)-defl),sens)+k2*defl*defl;
                else
                    if( ( (x)*(x)+(y)*(y) )>=(opt*opt) )
                        est1= k2*(x*x+y*y);
                    else
                        est1= k2*opt*opt;


            float est2;
            FindMin<float> min_est;
            for(int i=0;i<crit.points.size();i++)
            {
                float xx=crit.points[i].position.x-xPos;
                float yy=crit.points[i].position.y-yPos;

                float xG=crit.points[i].position.x-Gx;
                float yG=crit.points[i].position.y-Gy;

                float T2C_dist=sqrt(xG*xG+yG*yG);

                float ext_dist=max(K-T2C_dist,(float)0.0);

                min_est.iter(k1*(sqrt(xx*xx+yy*yy)-2*infl-ext_dist)+k2*(T2C_dist+ext_dist)*(T2C_dist+ext_dist));
            }
            est2=min_est.getVal();

            return max(est1,est2);
            //return est1;
        }
    }

    const T & staticCast(const float d) const
    {
        //cout<<d<<endl;
        static T r;
        r=static_cast<T>(d);
        //T r=d;
        //cout<<r<<endl;
        return r;
    }

    public:
        node(int xp, int yp, T d, T p, int inf, int def, T ss, float dist, bool q, bool bfs_, map_transform::VisNode cr_)
        {
            xPos=xp; yPos=yp; level=d; priority=p;infl=inf;defl=def;sens=ss;opt=dist;crit=cr_;
            power=q;
            bfs=bfs_;

            if(power)
            {
                costEst=&node::costEstimate2;
                costSens=&node::costSensing2;
                K=k1/(2*k2);
            }
            else
            {
                costEst=&node::costEstimate;
                costSens=&node::costSensing;
            }
        }

        int getxPos() const {return xPos;}
        int getyPos() const {return yPos;}
        T getLevel() const {return level;}
        T getPriority() const {return priority;}
        T getSensing() const {return sens;}

        void S2P(void)
        {
             priority=sens;
        }

        void updatePriority(const int & xDest, const int & yDest)
        {
            Gx=xDest;
            Gy=yDest;

            if(!bfs)
             priority=level+estimate(xDest, yDest)*1; //A*
            else
                priority=level; //A*


            //cout<<"Test bfs: "<<bfs<<"; "<<priority-level<<endl;
        }

        void updateSensing(const int & xDest, const int & yDest)
        {
            Gx=xDest;
            Gy=yDest;

            T ss=sensing(xDest, yDest);
            if(ss<0)
                sens=ss;
            else
                sens=level+ss; //A*
        }

        // give better priority to going strait instead of diagonally
        void nextLevel(const int & i) // i: direction
        {
             level+=k1*(dir==8?(i%2==0?10:14.14213562):10);
        }

        // Estimation function for the remaining distance to the goal.
        const T & estimate(const int & xDest, const int & yDest) const
        {
            static int xd, yd;
            static T d;
            xd=xDest-xPos;
            yd=yDest-yPos;

            // Euclidian Distance
            //d=static_cast<int>(costEstimate(xd,yd, infl, defl, opt));
            d=staticCast((this->*costEst)(xd,yd)*10.0);


            // Manhattan distance
            //d=abs(xd)+abs(yd);

            // Chebyshev distance
            //d=max(abs(xd), abs(yd));

            return(d);
        }

        const T & sensing(const int & xDest, const int & yDest) const
        {
            int xd, yd;
            static T d;
            xd=xDest-xPos;
            yd=yDest-yPos;

            // Euclidian Distance
            //d=static_cast<int>(costSensing(xd,yd, infl, defl, opt));
            d=staticCast((this->*costSens)(xd,yd)*10.0);

            //cout<<d<<endl;

            // Manhattan distance
            //d=abs(xd)+abs(yd);

            // Chebyshev distance
            //d=max(abs(xd), abs(yd));

            return(d);
        }

};

template <typename T>
bool operator<(const node<T> & a, const node<T> & b)
{
  return a.getPriority() > b.getPriority();
}


class nodeMP
{
    // current position
    int xPos;
    int yPos;
    // total distance already travelled to reach the node
    int level;
    // priority=level+remaining distance estimate
    int priority;  // smaller: higher priority

    public:
        nodeMP(int xp, int yp, int d, int p)
            {xPos=xp; yPos=yp; level=d; priority=p;}

        int getxPos() const {return xPos;}
        int getyPos() const {return yPos;}
        int getLevel() const  {return level;}
        int getPriority() const {return priority;}

        void updatePriority(const int xDest, const int yDest)
        {
             priority=level+estimate(xDest, yDest)*10; //A*
        }

        // give better priority to going strait instead of diagonally
        void nextLevel(const int i) // i: direction
        {
             level+=(dir==8?(i%2==0?10:14):10);
        }

        // Estimation function for the remaining distance to the goal.
        int estimate(const int xDest, const int yDest)
        {
            int xd, yd, d;
            xd=xDest-xPos;
            yd=yDest-yPos;

            // Euclidian Distance
            d=(int)(sqrt(xd*xd+yd*yd));

            // Manhattan distance
            //d=abs(xd)+abs(yd);

            // Chebyshev distance
            //d=max(abs(xd), abs(yd));

            return d;
        }

};

bool operator<(const nodeMP a,const nodeMP b)
{
  return a.getPriority() > b.getPriority();
}

class nodeMP0
{
    // current position
    int xPos;
    int yPos;
    // total distance already travelled to reach the node
    int level;
    // priority=level+remaining distance estimate
    int priority;  // smaller: higher priority

    public:
        nodeMP0(int xp, int yp, int d, int p)
            {xPos=xp; yPos=yp; level=d; priority=p;}

        int getxPos() const {return xPos;}
        int getyPos() const {return yPos;}
        int getLevel() const {return level;}
        int getPriority() const  {return priority;}

        void updatePriority(const int xDest,const int yDest)
        {
             priority=level+estimate(xDest, yDest)*0; //A*
        }

        // give better priority to going strait instead of diagonally
        void nextLevel(const int i) // i: direction
        {
             level+=(dir==8?(i%2==0?10:14):10);
        }

        // Estimation function for the remaining distance to the goal.
        int estimate(const int xDest,const int yDest)
        {
            int xd, yd, d;
            xd=xDest-xPos;
            yd=yDest-yPos;

            // Euclidian Distance
            d=(int)(sqrt(xd*xd+yd*yd));

            // Manhattan distance
            //d=abs(xd)+abs(yd);

            // Chebyshev distance
            //d=max(abs(xd), abs(yd));

            return(d);
        }

};

bool operator<(const nodeMP0 a,const nodeMP0 b)
{
  return a.getPriority() > b.getPriority();
}


class Planner{
private:
    int infl;
    int defl;

    nav_msgs::Path path_0, path_1;

    cv::Mat or_map;

    ros::NodeHandle nh_;

    ros::Publisher pub1, pub2;

    ros::Publisher pub_markers;
    
    ros::Subscriber sub1;

    ros::Subscriber sub2;

    ros::Subscriber sub3;

    ros::Subscriber graph_subscriber;

    ros::Subscriber sub_goals;

    tf::TransformListener pos_listener;

    std::string tf_pref;

    //std::vector<map_transform::VisNode> vis_;

    std::vector<float> vis_;

    std::vector<map_transform::VisNode> crit_points;

    void rcv_map1(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    void rcv_map2(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    void rcv_map3(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    void rcv_goal(const geometry_msgs::PoseStamped::ConstPtr& msg);

    ros::ServiceServer service, service3;

    ros::ServiceServer service2;

    vector<vector<vector<bool> > >  msg_rcv;

    vector<int> count;

    vector<bool> map_rcv;

    bool graph_rcv;

    bool pl;

    bool getMapValue(int n, int i, int j);

    vector<geometry_msgs::Point> goals;

    float res;

    int width,height;

    PointI convertW2I(geometry_msgs::Point p);

    geometry_msgs::Point convertI2W(PointI p);

    float convertCostI2W(float cost);

    template <typename T=float>
    Apath Astar(PointI p0, PointI p1, int r,   float opt=-3, bool bfs=false, map_transform::VisNode crit=map_transform::VisNode());

    Apath AstarMP(PointI p0, PointI p1, int r);
    Apath AstarMP0(PointI p0, PointI p1, int r);

    bool isGoal(PointI p0, PointI p1);

    void graphCallback(const map_transform::VisCom::ConstPtr& graph);

    void clearG();

    bool ask_plan(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

    bool clear(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

    bool request_single_plan(map_transform::PAstar::Request  &req, map_transform::PAstar::Response &res);

    bool planFromRequest(geometry_msgs::Point goal, float & cost, geometry_msgs::Point & perc_pt);
public:

    Planner(ros::NodeHandle nh): nh_(nh)
    {
        pub1 = nh_.advertise<nav_msgs::Path>("path0", 1,true);
        pub2 = nh_.advertise<nav_msgs::Path>("path1", 1,true);

        pub_markers = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1,true);

        sub1 = nh_.subscribe("v_map", 1, &Planner::rcv_map1, this);

        sub2 = nh_.subscribe("e_map", 1, &Planner::rcv_map2, this);

        sub3 = nh_.subscribe("/map", 1, &Planner::rcv_map3, this);

        sub_goals = nh_.subscribe("/move_base_simple/goal", 1, &Planner::rcv_goal, this);

        graph_subscriber = nh.subscribe("graph", 10 , &Planner::graphCallback, this);

        service = nh_.advertiseService("plan", &Planner::ask_plan, this);

        service3 = nh_.advertiseService("plan_point", &Planner::request_single_plan, this);

        service2 = nh_.advertiseService("clear", &Planner::clear, this);

        nh_.param("tf_prefix", tf_pref, std::string("/robot_0"));

        nh_.param("infl", infl, 5);
        nh_.param("defl", defl, infl);

        count.assign(3,0);

        map_rcv.assign(3,false);

        msg_rcv.resize(3);

        pl=false;
        graph_rcv=false;

        goals.clear();

    }

    ~Planner()
    {

    }

    void plan(void);

    void publish(void);
};

void Planner::graphCallback(const map_transform::VisCom::ConstPtr& graph)
{
    vis_=graph->vis;
    crit_points=graph->crit_points;
    graph_rcv=true;
}


void Planner::rcv_map1(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    msg_rcv[0].assign(msg->info.width, vector<bool>(msg->info.height,false));

    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
                if(*mapDataIterC == 0)
                {
                    msg_rcv[0][j][i]=true;
                }
                else
                {
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

void Planner::rcv_map2(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    msg_rcv[1].assign(msg->info.width, vector<bool>(msg->info.height,false));

    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
                if(*mapDataIterC == 0)
                {
                    msg_rcv[1][j][i]=true;
                }
                else
                {
                    msg_rcv[1][j][i]=false;
                }

                mapDataIterC++;
            }
    }

    map_rcv[1]=true;

    count[1]=count[1]+1;
}

void Planner::rcv_map3(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    msg_rcv[2].assign(msg->info.width, vector<bool>(msg->info.height,false));
    or_map = cv::Mat(msg->info.width, msg->info.height, CV_8UC1);

    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
                if(*mapDataIterC == 0)
                {
                    msg_rcv[2][j][i]=true;
                    or_map.at<uchar>(j,i) = 255;
                }
                else
                {
                    msg_rcv[2][j][i]=false;
                    or_map.at<uchar>(j,i) = 0;
                }

                mapDataIterC++;
            }
    }

    map_rcv[2]=true;

    count[2]=count[2]+1;
}



bool Planner::getMapValue(int n, int i, int j)
{
    return msg_rcv[n][i][j];
}

void Planner::clearG()
{
    pl=false;
    goals.clear();


    //nav_msgs::Path path_0,path_1;
    path_0.poses.clear();
    path_0.header.frame_id = "/map";
    path_0.header.stamp =  ros::Time::now();
    path_0.poses.clear();
    path_1=path_0;

    pub1.publish(path_0);
    pub2.publish(path_1);
}


void Planner::rcv_goal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //clearG();
    goals.push_back(msg->pose.position);
    pl=true;
}

bool Planner::ask_plan(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
    pl=true;
    return true;
}


bool Planner::request_single_plan(map_transform::PAstar::Request  &req, map_transform::PAstar::Response &res){
    geometry_msgs::Point perc_pt;
    float cost;
    planFromRequest(req.goal, cost, perc_pt);
    res.cost=cost;
    res.perc_pt=perc_pt;
    return true;
}

bool Planner::clear(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{

    clearG();
    return true;
}

PointI Planner::convertW2I(geometry_msgs::Point p)
{
    PointI pf(round(p.x/res),round(p.y/res));

    return pf;
}

geometry_msgs::Point Planner::convertI2W(PointI p)
{
    geometry_msgs::Point pf;
    pf.x=p.i*res;
    pf.y=p.j*res;

    return pf;
}

float Planner::convertCostI2W(float cost)
{
    return cost/10.0*res;
}

bool Planner::isGoal(PointI p0, PointI p1)
{
    //if(p0.i==p1.i && p0.j==p1.j)
    if( ( (p0.i-p1.i)*(p0.i-p1.i)+(p0.j-p1.j)*(p0.j-p1.j) )>(defl*defl) )
        return false;
    else
    {
        if(raytracing(or_map, p0.i, p0.j, p1.i, p1.j, true))
            return true;
        else
            return false;
    }
}


Apath Planner::AstarMP(PointI p0, PointI p1, int r)
{
    Apath path; path.points.clear();path.cost=0;
    const int n=width;
    const int m=height;
    vector<vector<int> > closed_nodes_map;closed_nodes_map.assign(n,vector<int>(m,0));
    vector<vector<int> > open_nodes_map;open_nodes_map.assign(n,vector<int>(m,0));
    vector<vector<int> > dir_map;dir_map.assign(n,vector<int>(m,0));

    priority_queue<nodeMP> pq[2];
    int pqi;
    nodeMP n0(p0.i, p0.j, 0, 0);

    int i, j, x, y, xdx, ydy;
    pqi=0;

    n0.updatePriority(p1.i, p1.j);
    pq[pqi].push(n0);

    while(!pq[pqi].empty())
    {

        n0=nodeMP( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                     pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x=n0.getxPos(); y=n0.getyPos();

        pq[pqi].pop();
        open_nodes_map[x][y]=0;

        if(closed_nodes_map[x][y]==1)
            continue;

        closed_nodes_map[x][y]=1;

        if(x==p1.i && y==p1.j)
        {
            while(!(x==p0.i && y==p0.j))
            {
                j=dir_map[x][y];

                if(j%2==0)
                    path.cost=path.cost+1;
                else
                    path.cost=path.cost+1.41421356237;
                path.points.insert(path.points.begin(),PointI(x,y));
                x+=dx[j];
                y+=dy[j];
            }

            //while(!pq[pqi].empty()) pq[pqi].pop();
            return path;
        }


        for(i=0;i<dir;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i];

            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || !msg_rcv[r][xdx][ydy]
                || closed_nodes_map[xdx][ydy]==1 ))//|| ( (dir==1 || dir==3 || dir==5 || dir==7)
                                                   //   && !msg_rcv[r][x+dx[(i-1)%dir]][y+dy[(i-1)%dir]]
                                                   //   && !msg_rcv[r][x+dx[(i+1)%dir]][y+dy[(i+1)%dir]]) ))
            {
                nodeMP m0( xdx, ydy, n0.getLevel(),
                             n0.getPriority());
                m0.nextLevel(i);
                m0.updatePriority(p1.i, p1.j);

                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0.getPriority();
                    pq[pqi].push(m0);
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0.getPriority())
                {
                    open_nodes_map[xdx][ydy]=m0.getPriority();

                    dir_map[xdx][ydy]=(i+dir/2)%dir;


                    //while(!(pq[pqi].top().getxPos()==xdx &&
                    //       pq[pqi].top().getyPos()==ydy))
                    //{
                    //    count++;
                    //    pq[1-pqi].push(pq[pqi].top());
                    //    pq[pqi].pop();
                    //}
                    //pq[pqi].pop();


                    //if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    //while(!pq[pqi].empty())
                    //{
                    //    pq[1-pqi].push(pq[pqi].top());
                    //    pq[pqi].pop();
                    //}
                    //pqi=1-pqi;
                    pq[pqi].push(m0);
                }
            }
        }
    }
    path.cost=-2;
    return path;
}


Apath Planner::AstarMP0(PointI p0, PointI p1, int r)
{
    Apath path; path.points.clear();path.cost=0;

    const int n=width;
    const int m=height;
    vector<vector<int> > closed_nodes_map;closed_nodes_map.assign(n,vector<int>(m,0));
    vector<vector<int> > open_nodes_map;open_nodes_map.assign(n,vector<int>(m,0));
    vector<vector<int> > dir_map;dir_map.assign(n,vector<int>(m,0));

    priority_queue<nodeMP0> pq[2];
    int pqi;
    nodeMP0 n0(p0.i, p0.j, 0, 0);
    int i, j, x, y, xdx, ydy;
    pqi=0;

    n0.updatePriority(p1.i, p1.j);
    pq[pqi].push(n0);

    long int exp_nodes=0, exp_nodes_r=0, tested_goal=0;

    while(!pq[pqi].empty())
    {
        exp_nodes++;

        n0=nodeMP0( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                     pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x=n0.getxPos(); y=n0.getyPos();

        pq[pqi].pop();
        open_nodes_map[x][y]=0;

        if(closed_nodes_map[x][y]==1)
            continue;

        closed_nodes_map[x][y]=1;

        if(x==p1.i && y==p1.j)
        {
            while(!(x==p0.i && y==p0.j))
            {
                j=dir_map[x][y];

                if(j%2==0)
                    path.cost=path.cost+1;
                else
                    path.cost=path.cost+1.41421356237;
                path.points.insert(path.points.begin(),PointI(x,y));
                x+=dx[j];
                y+=dy[j];
            }

            //while(!pq[pqi].empty()) pq[pqi].pop();
            cout<<"Expanded normal nodes: "<<exp_nodes<<endl;

            cout<<"Expanded backtrack nodes: "<<exp_nodes_r<<endl;

            cout<<"Goal Tested nodes: "<<tested_goal<<endl;


            return path;
        }


        for(i=0;i<dir;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i];

            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || !msg_rcv[r][xdx][ydy]
                || closed_nodes_map[xdx][ydy]==1 ))//|| ( (dir==1 || dir==3 || dir==5 || dir==7)
                                                   //   && !msg_rcv[r][x+dx[(i-1)%dir]][y+dy[(i-1)%dir]]
                                                   //   && !msg_rcv[r][x+dx[(i+1)%dir]][y+dy[(i+1)%dir]]) ))
            {
                nodeMP0 m0( xdx, ydy, n0.getLevel(),
                             n0.getPriority());
                m0.nextLevel(i);
                m0.updatePriority(p1.i, p1.j);

                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0.getPriority();
                    pq[pqi].push(m0);
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0.getPriority())
                {
                    open_nodes_map[xdx][ydy]=m0.getPriority();

                    dir_map[xdx][ydy]=(i+dir/2)%dir;


                    //while(!(pq[pqi].top().getxPos()==xdx &&
                    //       pq[pqi].top().getyPos()==ydy))
                    //{
                    //    count++;
                    //    pq[1-pqi].push(pq[pqi].top());
                    //    pq[pqi].pop();
                    //}
                    //pq[pqi].pop();


                    //if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    //while(!pq[pqi].empty())
                    //{
                    //    pq[1-pqi].push(pq[pqi].top());
                    //    pq[pqi].pop();
                    //}
                    //pqi=1-pqi;
                    pq[pqi].push(m0);
                }
            }
        }
    }

    cout<<"Expanded normal nodes: "<<exp_nodes<<endl;

    cout<<"Expanded backtrack nodes: "<<exp_nodes_r<<endl;

    cout<<"Goal Tested nodes: "<<tested_goal<<endl;

    path.cost=-2;
    return path;
}
int index_file=0;
ofstream myfile[14];

template <typename T>
Apath Planner::Astar(PointI p0, PointI p1, int map_index, float opt, bool bfs, map_transform::VisNode crit)
{

    //myfile << "Writing this to a file.\n";


    Apath path; path.points.clear();path.cost=0;

    const int n=width;
    const int m=height;
    vector<vector<int> > closed_nodes_map;closed_nodes_map.assign(n,vector<int>(m,0));
    vector<vector<int> > open_nodes_map;open_nodes_map.assign(n,vector<int>(m,0));
    vector<vector<int> > dir_map;dir_map.assign(n,vector<int>(m,0));

//    if(opt==-2)
//    {
//        if(msg_rcv[0][p1.i][p1.j] && !msg_rcv[1][p1.i][p1.j])
//        {
//            FindMin<float> crit;
//            bool found=false;

//            for(int x=max(p1.i-infl,0);x<min(p1.i+infl,(int)msg_rcv[0].size());x++)
//            {
//                for(int y=max(p1.j-infl,0);y<min(p1.j+infl,(int)msg_rcv[0][0].size());y++)
//                {
//                    if(msg_rcv[1][p1.i][p1.j])
//                    {
//                        found=true;
//                        float cost=(p1.i-x)*(p1.i-x)+(p1.j-y)*(p1.j-y);
//                        crit.iter(cost);
//                    }
//                }
//            }
//            if(found)
//                opt=sqrt(crit.getVal());
//        }
//    }

    priority_queue<node<T> > pq[3];
    int pqi;
    node<T> n0(p0.i, p0.j, 0, 0, infl, defl, 0, opt, quad, bfs, crit);
    int i, j, x, y, xdx, ydy;
    pqi=0;

    n0.updatePriority(p1.i, p1.j);
    n0.updateSensing(p1.i, p1.j);
    pq[pqi].push(n0);

    //static long int exp_nodes=0, exp_nodes_r=0, tested_goal=0;
    long int exp_nodes=0, exp_nodes_r=0, tested_goal=0;



    while(!pq[pqi].empty() || !pq[2].empty())
    {
        bool stop=false;

        bool list2=false;

        bool tested=false;


        if(pq[2].size()==0)
        {


            n0=node<T>( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                         pq[pqi].top().getLevel(), pq[pqi].top().getPriority(), infl, defl, pq[pqi].top().getSensing(), opt, quad, bfs, crit);
            x=n0.getxPos(); y=n0.getyPos();

            pq[pqi].pop();

            if(closed_nodes_map[x][y]==1)
                continue;

            //cout<<"From H"<<endl;

            if(n0.getSensing()>=0)
                if(n0.getPriority()>=n0.getSensing())
                {
                    tested_goal++;
                    tested=true;
                    if(isGoal(PointI(x,y),p1))
                        stop=true;
                }
        }
        else
        {
            if(pq[pqi].size()==0)
            {


                n0=node<T>( pq[2].top().getxPos(), pq[2].top().getyPos(),
                             pq[2].top().getLevel(), pq[2].top().getPriority(), infl, defl, pq[2].top().getSensing(), opt, quad, bfs, crit);
                x=n0.getxPos(); y=n0.getyPos();

                pq[2].pop();

                //if(closed_nodes_map[x][y]==1)
                //    continue;

                tested_goal++;

                list2=true;

                if(isGoal(PointI(x,y),p1))
                    stop=true;


                //cout<<"From R"<<endl;
            }
            else
            {
                bool cond=false;

                if(pq[2].top().getSensing()>=0)
                    if(pq[pqi].top().getPriority()>pq[2].top().getSensing())
                    {
                        cond=true;
                    }

                if(!cond)
                {
                    n0=node<T>( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                                 pq[pqi].top().getLevel(), pq[pqi].top().getPriority(), infl, defl, pq[pqi].top().getSensing(), opt, quad, bfs, crit);
                    x=n0.getxPos(); y=n0.getyPos();


                    pq[pqi].pop();

                    if(closed_nodes_map[x][y]==1)
                        continue;

                    if(n0.getSensing()>=0)
                        if(n0.getPriority()>=n0.getSensing())
                        {
                            tested_goal++;
                            tested=true;
                            if(isGoal(PointI(x,y),p1))
                                stop=true;
                        }

                    //cout<<"From H"<<endl;
                }
                else
                {
                    n0=node<T>( pq[2].top().getxPos(), pq[2].top().getyPos(),
                                 pq[2].top().getLevel(), pq[2].top().getPriority(), infl, defl, pq[2].top().getSensing(), opt, quad, bfs, crit);
                    x=n0.getxPos(); y=n0.getyPos();


                    pq[2].pop();

                    //if(closed_nodes_map[x][y]==1)
                    //    continue;

                    list2=true;

                    tested_goal++;

                    if(isGoal(PointI(x,y),p1))
                        stop=true;


                    //cout<<"From R"<<endl;
                }
            }
        }
        if(list2)
        {
            exp_nodes_r++;
            //cout<<x<<";"<<y<<"; "<<n0.getPriority()<<endl;
        }
        else
            exp_nodes++;



        //cout<<x<<"; "<<y<<"; "<<pq[0].size()<<"; "<<pq[1].size()<<"; "<<pq[2].size()<<endl;

        //if(pq[1].size()>1000000 || pq[0].size()>1000000)
        //{
        //    while(!pq[pqi].empty()) pq[pqi].pop();
        //    while(!pq[2].empty()) pq[2].pop();
        //    return path;
        //}




        //n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
        //             pq[pqi].top().getLevel(), pq[pqi].top().getPriority(), infl, defl, pq[pqi].top().getSensing());

        //x=n0->getxPos(); y=n0->getyPos();

        //pq[pqi].pop();
        open_nodes_map[x][y]=0;


        if(stop)
        {
            //cout<<n0->getSensing()<<": "<<sqrt((x-p1.i)*(x-p1.i)+(y-p1.j)*(y-p1.j))<<"; "<<opt<<endl;
            //cout<<n0->getLevel()<<endl;
            //cout<<n0->getxPos()<<";"<<n0->getyPos()<<endl;
            //cout<<p1.i<<";"<<p1.j<<endl;

            //T cost_test=0;

            while(!(x==p0.i && y==p0.j))
            {
                j=dir_map[x][y];

//                if(j%2==0)
//                    cost_test=cost_test+10;
//                 else
//                    cost_test=cost_test+14.1421356237;

                path.points.insert(path.points.begin(),PointI(x,y));
                x+=dx[j];
                y+=dy[j];
            }

            //cout<<cost_test*k1<<endl;
            path.points.insert(path.points.begin(),PointI(p0.i,p0.j));
            path.cost=(float)n0.getSensing();

            if(opt==-5)
            {
                exp_nodes=0; exp_nodes_r=0, tested_goal=0;
            }

            myfile[index_file]<<exp_nodes<<"; "<<exp_nodes_r<<"; "<<tested_goal<<"; ";


            cout<<"Expanded normal nodes: "<<exp_nodes<<endl;

            cout<<"Expanded backtrack nodes: "<<exp_nodes_r<<endl;

            cout<<"Goal Tested nodes: "<<tested_goal<<endl;

            //cout<<counter<<endl;



            return path;
        }


        if(list2)
            continue;

        if(closed_nodes_map[x][y]==1)
            continue;

        closed_nodes_map[x][y]=1;


        for(i=0;i<dir;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i];



            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 ))
            {
                if(!(!msg_rcv[map_index][xdx][ydy]
                    || closed_nodes_map[xdx][ydy]==1 //|| ( (dir==1 || dir==3 || dir==5 || dir==7)
                                                     //     && !msg_rcv[r][x+dx[(i-1)%dir]][y+dy[(i-1)%dir]]
                                                     //     && !msg_rcv[r][x+dx[(i+1)%dir]][y+dy[(i+1)%dir]])
                     ))
                {


                    node<T> m0=node<T>( xdx, ydy, n0.getLevel(),
                                 n0.getPriority(), infl, defl, n0.getSensing(), opt, quad, bfs, crit);
                    m0.nextLevel(i);
                    m0.updatePriority(p1.i, p1.j);
                    m0.updateSensing(p1.i, p1.j);

                    if(open_nodes_map[xdx][ydy]==0)
                    {
                        open_nodes_map[xdx][ydy]=m0.getPriority();
                        pq[pqi].push(m0);
                        dir_map[xdx][ydy]=(i+dir/2)%dir;

                    }
                    else if(open_nodes_map[xdx][ydy]>m0.getPriority())
                    {
                        open_nodes_map[xdx][ydy]=m0.getPriority();

                        dir_map[xdx][ydy]=(i+dir/2)%dir;


                        //while(!(pq[pqi].top().getxPos()==xdx &&
                        //       pq[pqi].top().getyPos()==ydy))
                        //{
                        //    pq[1-pqi].push(pq[pqi].top());
                        //    pq[pqi].pop();
                        //}
                        //pq[pqi].pop();


                        //if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                        //while(!pq[pqi].empty())
                        //{
                        //    pq[1-pqi].push(pq[pqi].top());
                        //    pq[pqi].pop();
                        //}
                        //pqi=1-pqi;
                        pq[pqi].push(m0);
                    }
                    //else delete m0;
                }
            }
        }


        if(n0.getSensing()>=0 && !tested)
        {
            n0.S2P();
            pq[2].push(n0);
        }


    }
    path.points.insert(path.points.begin(),PointI(p0.i,p0.j));
    path.cost=-2;
    //cout<<counter<<endl;

    if(opt==-5)
    {
        exp_nodes=0; exp_nodes_r=0, tested_goal=0;
    }


    cout<<"Expanded normal nodes: "<<exp_nodes<<endl;

    cout<<"Expanded backtrack nodes: "<<exp_nodes_r<<endl;

    cout<<"Goal Tested nodes: "<<tested_goal<<endl;

    //myfile[index_file]<<exp_nodes<<"; "<<exp_nodes_r<<"; "<<tested_goal<<"; ";

    return path;
}


void Planner::plan(void)
{
    //cout<<"TESTE!!!!!!!11111111"<<endl;
    //static int n=0;

//cout<<map_rcv[0]<<" "<< map_rcv[1]<<" "<<map_rcv[2] <<" "<<graph_rcv <<" "<< pl <<" "<< goals.size()<<endl;
    if(map_rcv[0] && map_rcv[1] && map_rcv[2] && graph_rcv && pl && (goals.size()>0) )
    {
        //cout<<"TESTE2222222222"<<endl;
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
        //PointI pi( (int) round((transform.getOrigin().x())/res), (int) round((transform.getOrigin().y())/res) );

        geometry_msgs::Point p;
        p.x=transform.getOrigin().x();
        p.y=transform.getOrigin().y();





        PointI pi;



        Apath path;


        //path=Astar(pi, convertW2I(goals[0]),1,-5,true);
        pl=false;

//        for(unsigned int tt=0;tt<2;tt++)
//        {



//            for(unsigned ll=0;ll<7;ll++)
//            {
//                k1=1;

//                if(ll==0)
//                    k2=0.008;
//                else
//                {k2*=5;}

//                if(tt==0)
//                    index_file=ll;
//                else
//                    index_file=7+ll;



//                if(tt==0)
//                {    quad=false;myfile[index_file]<<"Linear\n";}
//                else
//                {    quad=true;myfile[index_file]<<"Quadratic\n";}

//                myfile[index_file]<<"Lambda="<<k2/k1<<"\n";

//                for(int n=1;n<9;n++)
//                {

//                    string xsx;

//                    if(n==1)
//                    {
//                        p.x=10; p.y=10; xsx="sim_1";
//                    }
//                    else if(n==2)
//                    {
//                        p.x=8.3; p.y=7.5; xsx="sim_2"; //continue;
//                    }
//                    else if(n==3)
//                    {
//                        p.x=11.7; p.y=6.2; xsx="sim_3";//continue;
//                    }
//                    else if(n==4)
//                    {
//                        p.x=15; p.y=10.7; xsx="sim_4";//continue;
//                    }
//                    else if(n==5)
//                    {
//                        p.x=11.6; p.y=14; xsx="sim_5";//continue;
//                    }
//                    else if(n==6)
//                    {
//                        p.x=17.3; p.y=4.5; xsx="sim_6";//continue;
//                    }
//                    else if(n==7)
//                    {
//                        p.x=18.3; p.y=16.3; xsx="sim_7";//continue;
//                    }
//                    else if(n==8)
//                    {
//                        p.x=18.3; p.y=11.7; xsx="sim_8";//continue;
//                    }

                    //p.x=10; p.y=10;


                    pi=convertW2I(p);


                    //myfile[index_file]<<xsx<<"\n";
                    //cout<<xsx<<endl;

                    //cout<<n<<endl;


        for(unsigned int ii=0; ii<goals.size();ii++)//(goals.size()-1); i<goals.size();i++)//
        {
            ros::Time t01=ros::Time::now();
            ros::Duration diff;



            for(unsigned int i=goals.size(); i<ii+1;i++)//ii; i<ii+1;i++)//(goals.size()-1); i<goals.size();i++)//
            {

                PointI g=convertW2I(goals[i]);

                if(g.i<0 || g.i>=(int)msg_rcv[0].size())
                {
                    //clearG();
                    path.cost=-3;
                    continue;
                }
                if(g.j<0 || g.j>=(int)msg_rcv[0][g.i].size())
                {
                    //clearG();
                    path.cost=-3;
                    continue;
                }

                if(!msg_rcv[2][g.i][g.j])
                //if(!msg_rcv[1][g.i][g.j])
                {
                    //clearG();
                    path.cost=-3;
                    continue;
                }
    //t01=ros::Time::now();
                path=Astar(pi, g,1, -3, true);
                //path=AstarMP(pi, g,1);
                //cout<<path.cost<<endl;
            }

            diff = ros::Time::now() - t01;
            //if(path.cost!=-2)
            //    myfile[index_file]<<diff<<"; "<<path.cost<<"; ";


            ROS_INFO("Time BFS: %f; Cost: %f",diff.toSec(),path.cost);





            //path=Astar(pi, convertW2I(goals[0]),1,-5);

            t01=ros::Time::now();


            for(unsigned int i=ii; i<ii+1;i++)
            {

                PointI g=convertW2I(goals[i]);

                if(g.i<0 || g.i>=(int)msg_rcv[0].size())
                {
                    //clearG();
                    path.cost=-3;
                    continue;
                }
                if(g.j<0 || g.j>=(int)msg_rcv[0][g.i].size())
                {
                    //clearG();
                    path.cost=-3;
                    continue;
                }

                if(!msg_rcv[2][g.i][g.j])
                //if(!msg_rcv[1][g.i][g.j])
                {
                    //clearG();
                    path.cost=-3;
                    continue;
                }
    //t01=ros::Time::now();
                path=Astar(pi, g,1);
                //path=AstarMP0(pi, g,1);
                //cout<<path.cost<<endl;
            }


             diff = ros::Time::now() - t01;

            if(path.cost>=0)
                myfile[index_file]<<diff<<"; "<<path.cost<<"; ";

            ROS_INFO("Time PA: %f; Cost: %f",diff.toSec(),path.cost);

            path_0.poses.clear();

            path_1=path_0;

            if(path.cost>0)
            {
                //clearG();

                if(path.points.size()!=0)
                    for(unsigned int p_i=0;p_i<path.points.size();p_i++)
                    {
                        pw.pose.position=convertI2W(path.points[p_i]);

                        path_0.poses.push_back(pw);
                    }
                else
                {
                    pw.pose.position=p;
                    path_0.poses.push_back(pw);
                }
            }
    //        else if(path.cost!=-3)
    //        {
    //            pw.pose.position=p;
    //            path_0.poses.push_back(pw);
    //        }

            pub1.publish(path_0);




            //path=Astar(pi, convertW2I(goals[0]),1,-5);



            if(vis_.size()==(msg_rcv[0].size()*msg_rcv[0][0].size()))
            {
                t01=ros::Time::now();

                for(unsigned int i=ii; i<ii+1;i++)//goals.size(); i<goals.size();i++)//(goals.size()-1); i<goals.size();i++)//
                {

                    PointI g=convertW2I(goals[i]);

                    if(g.i<0 || g.i>=(int)msg_rcv[0].size())
                    {
                        //clearG();
                        path.cost=-3;
                        continue;
                    }
                    if(g.j<0 || g.j>=(int)msg_rcv[0][g.i].size())
                    {
                        //clearG();
                        path.cost=-3;
                        continue;
                    }

                    if(!msg_rcv[0][g.i][g.j])
                    //if(!msg_rcv[1][g.i][g.j])
                    {
                        //clearG();
                        path.cost=-3;
                        continue;
                    }
    //t01=ros::Time::now();
                    path=Astar(pi, g,1, vis_[g.i*msg_rcv[0][0].size()+g.j], false, crit_points[g.i*msg_rcv[0][0].size()+g.j]);
                    //path=AstarMP(pi, g,1);
                    //cout<<path.cost<<endl;
                }

                diff = ros::Time::now() - t01;

                ROS_INFO("Time PA-RDVM: %f; Cost: %f",diff.toSec(),path.cost);

                //if(path.cost!=-2)
                if(path.cost>=0)
                    myfile[index_file]<<diff<<"; "<<path.cost<<"; "<<"\n";

                if(path.cost>0)
                {
                    //clearG();

                    if(path.points.size()!=0)
                        for(unsigned int p_i=0;p_i<path.points.size();p_i++)
                        {
                            pw.pose.position=convertI2W(path.points[p_i]);

                            path_1.poses.push_back(pw);
                        }
                    else
                    {
                        pw.pose.position=p;
                        path_1.poses.push_back(pw);
                    }
               }
        //        else if(path.cost!=-3)
        //        {

        //        }

                pub2.publish(path_1);
            }




       // pl=false;
    }
    }
//            }
//        }
//    }
}



void Planner::publish(void)
{
    visualization_msgs::MarkerArray points;
    visualization_msgs::Marker point;

    if(goals.size()>0)
    {
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

        for (uint32_t i = 0; i < goals.size(); ++i)
        {
          point.pose.position=goals[i];
          point.id = i;

          points.markers.push_back(point);
        }

        if(path_0.poses.size()>0)
        {
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

        if(path_1.poses.size()>0)
        {
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
    else
    {
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
    }

    pub_markers.publish(points);
}

bool Planner::planFromRequest(geometry_msgs::Point goal, float & cost, geometry_msgs::Point & perc_pt){
    if(map_rcv[0] && map_rcv[1] && map_rcv[2] && graph_rcv){
        path_0.poses.clear();
        path_0.header.frame_id = "/map";
        path_0.header.stamp =  ros::Time::now();
        geometry_msgs::PoseStamped pw;
        pw.header.frame_id   = "/map";
        pw.header.stamp =  ros::Time::now();
        pw.pose.orientation.w=1;

        PointI gi=convertW2I(goal);

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
        PointI pi=convertW2I(pt);

        //Astar(pi, gi,1,-5);
        Apath path=Astar(pi, gi,1);
        if(path.cost>=0){
            path_0.poses.clear();
            if(path.points.size()!=0){
                for(unsigned int p_i=0;p_i<path.points.size();p_i++){
                    pw.pose.position=convertI2W(path.points[p_i]);
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

  myfile[0].open ("/home/viki/rosbuild_ws/map_transform/resultsL1.txt", ios::out);
  myfile[1].open ("/home/viki/rosbuild_ws/map_transform/resultsL2.txt", ios::out);
  myfile[2].open ("/home/viki/rosbuild_ws/map_transform/resultsL3.txt", ios::out);
  myfile[3].open ("/home/viki/rosbuild_ws/map_transform/resultsL4.txt", ios::out);
  myfile[4].open ("/home/viki/rosbuild_ws/map_transform/resultsL5.txt", ios::out);
  myfile[5].open ("/home/viki/rosbuild_ws/map_transform/resultsL6.txt", ios::out);
  myfile[6].open ("/home/viki/rosbuild_ws/map_transform/resultsL7.txt", ios::out);

  myfile[7].open ("/home/viki/rosbuild_ws/map_transform/resultsQ1.txt", ios::out);
  myfile[8].open ("/home/viki/rosbuild_ws/map_transform/resultsQ2.txt", ios::out);
  myfile[9].open ("/home/viki/rosbuild_ws/map_transform/resultsQ3.txt", ios::out);
  myfile[10].open ("/home/viki/rosbuild_ws/map_transform/resultsQ4.txt", ios::out);
  myfile[11].open ("/home/viki/rosbuild_ws/map_transform/resultsQ5.txt", ios::out);
  myfile[12].open ("/home/viki/rosbuild_ws/map_transform/resultsQ6.txt", ios::out);
  myfile[13].open ("/home/viki/rosbuild_ws/map_transform/resultsQ7.txt", ios::out);

  Planner planner(nh);

  ros::Rate loop_rate(10);

  //node<float> n0(1,1,0,0,8,80,0,20,false);
  //n0.nextLevel(1);
  //n0.updateSensing(4,4);
  //cout<<n0.getxPos()+dx[1]<<" "<<n0.getyPos()+dy[1]<<" "<<n0.getSensing()<<endl;

  //cout<<static_cast<float>(5.555)<<endl;
  while (ros::ok())
  {


    ros::spinOnce();

    planner.plan();

    planner.publish();

    loop_rate.sleep();
  }

  for(int i=0;i<14;i++)
   myfile[i].close();

  return 0;
}
