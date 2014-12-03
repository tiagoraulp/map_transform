#include "ros/ros.h"
#include <vector>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_listener.h"

#include "std_msgs/String.h"

#include "std_srvs/Empty.h"

using namespace std;

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


class Planner{
private:
    ros::NodeHandle nh_;

    ros::Publisher pub1;
    ros::Publisher pub2;

    ros::Publisher pub_markers;
    
    ros::Subscriber sub1;

    ros::Subscriber sub2;

    ros::ServiceServer service;

    ros::Subscriber sub_goals;

    tf::TransformListener pos_listener;

    void rcv_map1(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    void rcv_map2(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    bool ask_plan(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

    void rcv_goal(const geometry_msgs::PoseStamped::ConstPtr& msg);

    vector<vector<vector<bool> > >  msg_rcv;

    vector<int> count;

    vector<bool> map_rcv;

    bool pl;

    unsigned char getMapValue(int n, int i, int j);

    vector<geometry_msgs::Point> goals;

    int prev_goals;

    float res;

    PointI convertW2I(geometry_msgs::Point p);

    geometry_msgs::Point convertI2W(PointI p);


public:



    Planner(ros::NodeHandle nh): nh_(nh)
    {

        pub1 = nh_.advertise<nav_msgs::Path>("path0", 1,true);
        pub2 = nh_.advertise<nav_msgs::Path>("path1", 1,true);

        pub_markers = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1,true);

        sub1 = nh_.subscribe("/robot_0/l_map", 1, &Planner::rcv_map1, this);

        sub2 = nh_.subscribe("/robot_1/l_map", 1, &Planner::rcv_map2, this);


        service = nh_.advertiseService("plan", &Planner::ask_plan, this);

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


unsigned char Planner::getMapValue(int n, int i, int j)
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

void Planner::plan(void)
{
    if(map_rcv[0] && map_rcv[1] && pl && (goals.size()>0) )
    {
        nav_msgs::Path path_0,path_1;

        vector<PointI> p0,p1;

        vector<PointI> g,g0,g1,gb,gn;

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
                g0.push_back(pi_temp);
            else if ( msg_rcv[1][pi_temp.i][pi_temp.j] && !msg_rcv[0][pi_temp.i][pi_temp.j] )
                g1.push_back(pi_temp);
            else if ( msg_rcv[1][pi_temp.i][pi_temp.j] && msg_rcv[0][pi_temp.i][pi_temp.j] )
                gb.push_back(pi_temp);
            else
                gn.push_back(pi_temp);

        }
        cout<<"TEest!!!!"<<endl;
        for(int i=0;i<g0.size();i++)
        {
            cout<<g0[i].i<<" "<<g0[i].j<<endl;

            p_temp.pose.position=convertI2W(g0[i]);
            path_0.poses.push_back(p_temp);

        }
        for(int i=0;i<g1.size();i++)
        {
            cout<<g1[i].i<<" "<<g1[i].j<<endl;
            p_temp.pose.position=convertI2W(g1[i]);
            path_1.poses.push_back(p_temp);

        }

        for(int i=0;i<gb.size();i++)
        {
            cout<<gb[i].i<<" "<<gb[i].j<<endl;
            p_temp.pose.position=convertI2W(gb[i]);
            path_0.poses.push_back(p_temp);

        }
        cout<<"DONEEEEEEEEEEE!!!!"<<endl;



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

        point.lifetime = ros::Duration(0.1);


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
