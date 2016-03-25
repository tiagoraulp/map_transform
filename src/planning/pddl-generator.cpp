#include "ros/ros.h"
#include <ros/package.h>
#include <signal.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <opencv2/core/core.hpp>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_listener.h"

using namespace std;

class PddlGen{
private:
    ros::NodeHandle nh_;

    ros::Subscriber sub1, sub2, sub3, sub4;

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
    bool getMapValue(int n, int i, int j);
    cv::Point2i convertW2I(geometry_msgs::Point p);
    geometry_msgs::Point convertI2W(cv::Point2i p);

public:
    PddlGen(ros::NodeHandle nh): nh_(nh)
    {
        sub1 = nh_.subscribe("/robot_0/v_map", 1, &PddlGen::rcv_map1, this);
        sub2 = nh_.subscribe("/robot_1/v_map", 1, &PddlGen::rcv_map2, this);
        sub3 = nh_.subscribe("/robot_0/e_map", 1, &PddlGen::rcv_map3, this);
        sub4 = nh_.subscribe("/robot_1/e_map", 1, &PddlGen::rcv_map4, this);
        count.assign(4,0);
        map_rcv.assign(4,false);
        msg_rcv.resize(4);

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

void write_preamble(stringstream & str, int nr, int nw)
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
        str<<"waypoint"<<i<<" ";
    }
    str<<"- waypoint"<<endl<<")"<<endl<<endl;
}

void write_graph(stringstream & str)
{
    str<<"(:init\n";

    str<<"\t(connected waypoint0 waypoint1)"<<endl;

    str<<")"<<endl<<endl;
}

void write_goals(stringstream & str)
{
    str<<"(:goal (and"<<endl;

    str<<"\t(visited waypoint0)"<<endl;

    str<<"\t)\n)\n)";
}


bool PddlGen::plan(void)
{
    if(map_rcv[0] && map_rcv[1] && map_rcv[2] && map_rcv[3])
    {
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

        //cout<<pos_r0.x<<" "<<pos_r0.y<<" "<<pos_r1.x<<" "<<pos_r1.y<<" "<<endl;

        stringstream strstream("");
        write_preamble(strstream, 2,5);
        write_graph(strstream);
        write_goals(strstream);

        ofstream myfile;
        string pddlFolder = ros::package::getPath("map_transform").append("/pddl/problem.pddl");
        myfile.open (pddlFolder);
        if (myfile.is_open())
        {
            myfile << strstream.str();
        }
        else
        {
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
