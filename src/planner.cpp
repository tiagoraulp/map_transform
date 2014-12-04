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
        cost=0;
        points.clear();
    }
};


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

    Apath Astar(PointI p0, PointI p1, int r);


public:



    Planner(ros::NodeHandle nh): nh_(nh)
    {

        pub1 = nh_.advertise<nav_msgs::Path>("path0", 1,true);
        pub2 = nh_.advertise<nav_msgs::Path>("path1", 1,true);

        pub_markers = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1,true);

        sub1 = nh_.subscribe("/robot_0/l_map", 1, &Planner::rcv_map1, this);

        sub2 = nh_.subscribe("/robot_1/l_map", 1, &Planner::rcv_map2, this);


        service = nh_.advertiseService("plan", &Planner::ask_plan, this);

        service2 = nh_.advertiseService("clear", &Planner::clear, this);

        sub_goals = nh_.subscribe("/move_base_simple/goal", 1, &Planner::rcv_goal, this);

        count.resize(2,0);

        map_rcv.resize(2,false);

        msg_rcv.resize(2);

        pl=false;

        goals.clear();

        prev_goals=goals.size();

    }

    ~Planner()
    {

    }

    void plan(void);

    void publish(void);




};


void Planner::rcv_map1(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    //boost::mutex::scoped_lock lock(mux);
    //ROS_INFO("I heard map 1: [%d]", msg->header.seq);

    //msg_rcv[0]=*msg;
    msg_rcv[0].resize(msg->info.width, vector<bool>(msg->info.height,false));

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
    //boost::mutex::scoped_lock lock(mux);
    //ROS_INFO("I heard map 2: [%d]", msg->header.seq);

    //msg_rcv[1]=*msg;
    msg_rcv[1].resize(msg->info.width, vector<bool>(msg->info.height,false));

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


bool Planner::getMapValue(int n, int i, int j)
{
    //int width= msg_rcv[n].info.width;


    //std::vector<signed char>::const_iterator mapDataIterC = msg_rcv[n].data.begin();

    //mapDataIterC=mapDataIterC+i*width+j;

    //return *mapDataIterC;
    return msg_rcv[n][i][j];

}

bool Planner::ask_plan(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
    pl=true;
    return true;
}


bool Planner::clear(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
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

    pub2.publish(path_1);

    return true;
}

void Planner::rcv_goal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goals.push_back(msg->pose.position);

}

PointI Planner::convertW2I(geometry_msgs::Point p)
{
    PointI pf(p.x/res,p.y/res);

    return pf;
}

geometry_msgs::Point Planner::convertI2W(PointI p)
{
    geometry_msgs::Point pf;
    pf.x=p.i*res;
    pf.y=p.j*res;

    return pf;
}

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

        // Determine priority (in the priority queue)

};

bool operator<(const node & a, const node & b)
{
  return a.getPriority() > b.getPriority();
}


Apath Planner::Astar(PointI p0, PointI p1, int r)
{
    // Astar.cpp
    // http://en.wikipedia.org/wiki/A*
    // Compiler: Dev-C++ 4.9.9.2
    // FB - 201012256
    //#include <iostream>
    //#include <iomanip>
    //#include <queue>
    //#include <string>
    //#include <math.h>
    //#include <ctime>
    //using namespace std;

    Apath path; path.points.clear();

    const int n=width; // horizontal size of the map
    const int m=height; // vertical size size of the map
    //static int map[n][m];
    vector<vector<int> > closed_nodes_map;closed_nodes_map.resize(n,vector<int>(m,0)); // map of closed (tried-out) nodes
    vector<vector<int> > open_nodes_map;open_nodes_map.resize(n,vector<int>(m,0)); // map of open (not-yet-tried) nodes
    vector<vector<int> > dir_map;dir_map.resize(n,vector<int>(m,0)); // map of directions





// A-star algorithm.
// The route returned is a string of direction digits.

    static priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
    static int pqi; // pq index
    static node* n0;
    static node* m0;
    static int i, j, x, y, xdx, ydy;
    pqi=0;

    // reset the node maps
    //for(y=0;y<m;y++)
    //{
    //    for(x=0;x<n;x++)
    //    {
    //       closed_nodes_map[x][y]=0;
    //        open_nodes_map[x][y]=0;
    //    }
    //}

    // create the start node and push into list of open nodes
    n0=new node(p0.i, p0.j, 0, 0);
    n0->updatePriority(p1.i, p1.j);
    pq[pqi].push(*n0);
    //open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map

    // A* search
    while(!pq[pqi].empty())
    {
        // get the current node w/ the highest priority
        // from the list of open nodes
        n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                     pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x=n0->getxPos(); y=n0->getyPos();

        pq[pqi].pop(); // remove the node from the open list
        open_nodes_map[x][y]=0;
        // mark it on the closed nodes map
        closed_nodes_map[x][y]=1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if(x==p1.i && y==p1.j)
        {
            // generate the path from finish to start
            // by following the directions
            //string path="";
            while(!(x==p0.i && y==p0.j))
            {
                j=dir_map[x][y];
                //c='0'+(j+dir/2)%dir;
                //path=c+path;
                if(j%2==0)
                    path.cost=path.cost+1;
                else
                    path.cost=path.cost+1.41421356237;
                path.points.insert(path.points.begin(),PointI(x,y));
                x+=dx[j];
                y+=dy[j];
            }

            // garbage collection
            delete n0;
            // empty the leftover nodes
            while(!pq[pqi].empty()) pq[pqi].pop();
            return path;
        }

        // generate moves (child nodes) in all possible directions
        for(i=0;i<dir;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i];

            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || !msg_rcv[r][xdx][ydy]
                || closed_nodes_map[xdx][ydy]==1 || ( (dir==1 || dir==3 || dir==5 || dir==7)
                                                      && !msg_rcv[r][x+dx[(i-1)%dir]][y+dy[(i-1)%dir]]
                                                      && !msg_rcv[r][x+dx[(i+1)%dir]][y+dy[(i+1)%dir]]) ))
            {
                // generate a child node
                m0=new node( xdx, ydy, n0->getLevel(),
                             n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(p1.i, p1.j);

                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
                    // mark its parent node direction
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
                    // update the priority info
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy]=(i+dir/2)%dir;

                    // replace the node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(pq[pqi].top().getxPos()==xdx &&
                           pq[pqi].top().getyPos()==ydy))
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pq[pqi].pop(); // remove the wanted node

                    // empty the larger size pq to the smaller one
                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*m0); // add the better node instead
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    return path; // no route found
}



void Planner::plan(void)
{
    if(map_rcv[0] && map_rcv[1] && pl && (goals.size()>0) )
    {
        nav_msgs::Path path_0,path_1;

        vector<PointI> p0,p1;

        vector<int> p0g,p1g,g0,g1,gb,gn;

        vector<PointI> g;

        p0.clear();p1.clear();

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

        int x_0=(int) round((transform.getOrigin().x())/res);

        int y_0=(int) round((transform.getOrigin().y())/res);

        PointI pi_temp( (int) round((transform.getOrigin().x())/res), (int) round((transform.getOrigin().y())/res) );

        p_temp.pose.position=convertI2W(pi_temp);

        path_0.poses.push_back(p_temp);

        p0.push_back(pi_temp);


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

        p1.push_back(pi_temp);


        for(int i=0;i<goals.size();i++)
        {
            p_temp.pose.position=goals[i];

            //path_0.poses.push_back(p_temp);

            pi_temp=convertW2I(p_temp.pose.position);

            g.push_back(pi_temp);

            if( msg_rcv[0][pi_temp.i][pi_temp.j] && !msg_rcv[1][pi_temp.i][pi_temp.j] )
                g0.push_back(i);
            else if ( msg_rcv[1][pi_temp.i][pi_temp.j] && !msg_rcv[0][pi_temp.i][pi_temp.j] )
                g1.push_back(i);
            else if ( msg_rcv[1][pi_temp.i][pi_temp.j] && msg_rcv[0][pi_temp.i][pi_temp.j] )
                gb.push_back(i);
            else
                gn.push_back(i);

        }

        int g0s=g0.size(), g1s=g1.size(),gbs=gb.size();

        vector<vector<Apath> > mat(2+g.size(),vector<Apath>(2+g.size(),Apath()));


        Apath path;
        if(g0s>0)
        {
            double max_cost=g0[0];
            for(int i=0;i<g0.size();i++)
            {
                path=Astar(p0.front(), g0[i],0);

                mat[0][2+i]=path;
                mat[2+i][0]=path;

                if(path.cost>mat[0][2+g[max_cost]])
                    max_cost=i;

            }

            p0g.push_back(g[max_cost]);
        }

        if(g1s>0)
        {
            double max_cost=g1[0];
            for(int i=0;i<g1.size();i++)
            {
                path=Astar(p1.front(), g1[i],0);

                mat[1][2+i]=path;
                mat[2+i][1]=path;

                if(path.cost>mat[1][2+g[max_cost]])
                    max_cost=i;

            }

            p1g.push_back(g[max_cost]);
        }


        //<<"TEest!!!!"<<endl;
        //vector<PointI> path;

//        for(int i=0;i<g0.size();i++)
//        {
//            //cout<<g0[i].i<<" "<<g0[i].j<<endl;
//            path=Astar(p0.back(), g[g0[i]],0);
//            //cout<<path.size();
//            for(int p_i=0;p_i<path.points.size();p_i++)
//            {


//                p0.push_back(path.points[p_i]);
//            }



//        }
//        for(int i=0;i<g1.size();i++)
//        {
//            //cout<<g1[i].i<<" "<<g1[i].j<<endl;
//            path=Astar(p1.back(), g[g1[i]],1);
//            //cout<<path.size();
//            for(int p_i=0;p_i<path.points.size();p_i++)
//            {


//                p1.push_back(path.points[p_i]);
//            }

//            //p_temp.pose.position=convertI2W(g1[i]);
//            //path_1.poses.push_back(p_temp);

//        }

//        for(int i=0;i<gb.size();i++)
//        {
//            path=Astar(p0.back(), g[gb[i]],0);
//            //cout<<path.size();
//            for(int p_i=0;p_i<path.points.size();p_i++)
//            {
//                p0.push_back(path.points[p_i]);
//            }
//            //cout<<gb[i].i<<" "<<gb[i].j<<endl;
//            //p_temp.pose.position=convertI2W(gb[i]);
//            //path_0.poses.push_back(p_temp);

//        }


        //cout<<"DONEEEEEEEEEEE!!!!"<<endl;

        for(int p_i=0;p_i<p0g.size();p_i++)
        {
            p0.push_back();

            path_0.poses.push_back(p_temp);
        }


        path_0.poses.clear();
        for(int p_i=0;p_i<p0.size();p_i++)
        {
            p_temp.pose.position=convertI2W(p0[p_i]);

            path_0.poses.push_back(p_temp);
        }

        path_1.poses.clear();
        for(int p_i=0;p_i<p1.size();p_i++)
        {
            p_temp.pose.position=convertI2W(p1[p_i]);

            path_1.poses.push_back(p_temp);
        }



        pub1.publish(path_0);

        pub2.publish(path_1);


    }
}

void Planner::publish(void)
{
    if(goals.size()>0)
    {

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

        // Points are green
        point.color.g = 1.0f;
        point.color.a = 1.0;

        point.lifetime = ros::Duration(2);


        for (uint32_t i = 0; i < goals.size(); ++i)
        {
          point.pose.position=goals[i];
          point.id = i;

          points.markers.push_back(point);
        }



        pub_markers.publish(points);

        prev_goals=goals.size();

    }
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
