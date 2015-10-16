#include "ros/ros.h"
#include <vector>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_listener.h"
#include <map_transform/VisCom.h>

#include <queue>
#include "std_msgs/String.h"

#include "std_srvs/Empty.h"

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
    double cost;

    Apath()
    {
        cost=-1;
        points.clear();
    }
};

class node
{
    // current position
    int xPos;
    int yPos;
    // total distance already travelled to reach the node
    int level;
    // priority=level+remaining distance estimate
    int priority;  // smaller: higher priority

    public:
        node(int xp, int yp, int d, int p)
            {xPos=xp; yPos=yp; level=d; priority=p;}

        int getxPos() const {return xPos;}
        int getyPos() const {return yPos;}
        int getLevel() const {return level;}
        int getPriority() const {return priority;}

        void updatePriority(const int & xDest, const int & yDest)
        {
             priority=level+estimate(xDest, yDest)*10; //A*
        }

        // give better priority to going strait instead of diagonally
        void nextLevel(const int & i) // i: direction
        {
             level+=(dir==8?(i%2==0?10:14):10);
        }

        // Estimation function for the remaining distance to the goal.
        const int & estimate(const int & xDest, const int & yDest) const
        {
            static int xd, yd, d;
            xd=xDest-xPos;
            yd=yDest-yPos;

            // Euclidian Distance
            d=static_cast<int>(sqrt(xd*xd+yd*yd));

            // Manhattan distance
            //d=abs(xd)+abs(yd);

            // Chebyshev distance
            //d=max(abs(xd), abs(yd));

            return(d);
        }

};

bool operator<(const node & a, const node & b)
{
  return a.getPriority() > b.getPriority();
}


class Planner{
private:
    ros::NodeHandle nh_;

    ros::Publisher pub1;

    ros::Publisher pub_markers;
    
    ros::Subscriber sub1;

    ros::Subscriber sub2;

    ros::Subscriber sub3;

    ros::Subscriber graph_subscriber;

    ros::Subscriber sub_goals;

    tf::TransformListener pos_listener;

    std::vector<map_transform::VisNode> vis_;

    void rcv_map1(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    void rcv_map2(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    void rcv_map3(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    void rcv_goal(const geometry_msgs::PoseStamped::ConstPtr& msg);

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

    Apath Astar(PointI p0, PointI p1, int r);

    void graphCallback(const map_transform::VisCom::ConstPtr& graph);

    void clearG();

public:

    Planner(ros::NodeHandle nh): nh_(nh)
    {

        pub1 = nh_.advertise<nav_msgs::Path>("path0", 1,true);

        pub_markers = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1,true);

        sub1 = nh_.subscribe("/robot_0/v_map", 1, &Planner::rcv_map1, this);

        sub2 = nh_.subscribe("/robot_0/e_map", 1, &Planner::rcv_map2, this);

        sub3 = nh_.subscribe("/map", 1, &Planner::rcv_map3, this);

        sub_goals = nh_.subscribe("/move_base_simple/goal", 1, &Planner::rcv_goal, this);

        graph_subscriber = nh.subscribe("graph", 10 , &Planner::graphCallback, this);

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

    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
                if(*mapDataIterC == 0)
                {
                    msg_rcv[2][j][i]=true;
                }
                else
                {
                    msg_rcv[2][j][i]=false;
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

    nav_msgs::Path path_0,path_1;
    path_0.poses.clear();
    path_0.header.frame_id = "/map";
    path_0.header.stamp =  ros::Time::now();
    path_0.poses.clear();
    path_1=path_0;

    pub1.publish(path_0);
}


void Planner::rcv_goal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    clearG();
    goals.push_back(msg->pose.position);
    pl=true;
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

Apath Planner::Astar(PointI p0, PointI p1, int r)
{
    Apath path; path.points.clear();path.cost=0;

    const int n=width;
    const int m=height;
    vector<vector<int> > closed_nodes_map;closed_nodes_map.assign(n,vector<int>(m,0));
    vector<vector<int> > open_nodes_map;open_nodes_map.assign(n,vector<int>(m,0));
    vector<vector<int> > dir_map;dir_map.assign(n,vector<int>(m,0));

    static priority_queue<node> pq[2];
    static int pqi;
    static node* n0;
    static node* m0;
    static int i, j, x, y, xdx, ydy;
    pqi=0;

    n0=new node(p0.i, p0.j, 0, 0);
    n0->updatePriority(p1.i, p1.j);
    pq[pqi].push(*n0);

    while(!pq[pqi].empty())
    {
        n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                     pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x=n0->getxPos(); y=n0->getyPos();

        pq[pqi].pop();
        open_nodes_map[x][y]=0;

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
            path.points.insert(path.points.begin(),PointI(p0.i,p0.j));

            delete n0;

            while(!pq[pqi].empty()) pq[pqi].pop();
            return path;
        }


        for(i=0;i<dir;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i];

            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || !msg_rcv[r][xdx][ydy]
                || closed_nodes_map[xdx][ydy]==1 || ( (dir==1 || dir==3 || dir==5 || dir==7)
                                                      && !msg_rcv[r][x+dx[(i-1)%dir]][y+dy[(i-1)%dir]]
                                                      && !msg_rcv[r][x+dx[(i+1)%dir]][y+dy[(i+1)%dir]]) ))
            {
                m0=new node( xdx, ydy, n0->getLevel(),
                             n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(p1.i, p1.j);

                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();

                    dir_map[xdx][ydy]=(i+dir/2)%dir;


                    while(!(pq[pqi].top().getxPos()==xdx &&
                           pq[pqi].top().getyPos()==ydy))
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pq[pqi].pop();


                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*m0);
                }
                else delete m0;
            }
        }
        delete n0;
    }
    path.cost=-2;
    return path;
}


double cost_function(double a, double b)
{
    return max(a,b);
}

void Planner::plan(void)
{
    if(map_rcv[0] && map_rcv[1] && graph_rcv && pl && (goals.size()>0) )
    {
        nav_msgs::Path path_0;

        PointI g=convertW2I(goals[0]);

        if(g.i<0 || g.i>=(int)msg_rcv[0].size())
        {
            clearG();
            return;
        }
        if(g.j<0 || g.j>=(int)msg_rcv[0][g.i].size())
        {
            clearG();
            return;
        }
        if(!msg_rcv[0][g.i][g.j])
        {
            clearG();
            return;
        }

        path_0.poses.clear();
        path_0.header.frame_id = "/map";
        path_0.header.stamp =  ros::Time::now();

        geometry_msgs::PoseStamped pw;
        pw.header.frame_id   = "/map";
        pw.header.stamp =  ros::Time::now();

        pw.pose.orientation.w=1;

        tf::StampedTransform transform;
        try{
            pos_listener.lookupTransform("/map", "/robot_0/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_INFO("%s",ex.what());
          return ;
        }

        //PointI pi( (int) round((transform.getOrigin().x())/res), (int) round((transform.getOrigin().y())/res) );

        geometry_msgs::Point p;
        p.x=transform.getOrigin().x();
        p.y=transform.getOrigin().y();

        PointI pi=convertW2I(p);

        Apath path;

        path=Astar(pi, g,1);

        ROS_INFO("%f", path.cost);


        path_0.poses.clear();
        for(unsigned int p_i=0;p_i<path.points.size();p_i++)
        {
            pw.pose.position=convertI2W(path.points[p_i]);

            path_0.poses.push_back(pw);
        }

        pub1.publish(path_0);

        pl=false;
    }
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

        point.type=visualization_msgs::Marker::LINE_STRIP;
        point.color.r = 1.0f;
        point.color.g = 0.0;
        point.scale.x = 0.03;
        point.scale.y = 0.02;
        point.scale.z = 0.001;
        geometry_msgs::Point p;
        p.x=0.0f; p.y=0.0f; p.z=0.0f;
        point.points.clear();
        point.points.push_back(p);
        point.points.push_back(goals[0]);
        point.pose.position=p;
        point.id = goals.size();
        points.markers.push_back(point);
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


int main(int argc, char **argv)
{

  
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh("~");


  Planner planner(nh);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {


    ros::spinOnce();

    planner.plan();

    planner.publish();

    loop_rate.sleep();
  }


  return 0;
}
