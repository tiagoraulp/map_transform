#include <signal.h>
#include "ros/ros.h"
#include <vector>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_listener.h"
#include <opencv2/core/core.hpp>

using namespace std;

class Planner{
private:
    ros::NodeHandle nh_;

    ros::Subscriber sub1;
    ros::Subscriber sub2;

    vector<vector<vector<bool> > >  msg_rcv;
    vector<int> count;
    vector<bool> map_rcv;
    float res;
    int width,height;

    tf::TransformListener pos_listener;

    void rcv_map1(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void rcv_map2(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    bool getMapValue(int n, int i, int j);
    cv::Point2i convertW2I(geometry_msgs::Point p);
    geometry_msgs::Point convertI2W(cv::Point2i p);

public:
    Planner(ros::NodeHandle nh): nh_(nh)
    {
        sub1 = nh_.subscribe("/robot_0/v_map", 1, &Planner::rcv_map1, this);
        sub2 = nh_.subscribe("/robot_1/v_map", 1, &Planner::rcv_map2, this);
        count.assign(2,0);
        map_rcv.assign(2,false);
        msg_rcv.resize(2);
    }

    ~Planner()
    {
    }

    bool plan(void);
};

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

bool Planner::getMapValue(int n, int i, int j)
{
    return msg_rcv[n][i][j];
}

cv::Point2i Planner::convertW2I(geometry_msgs::Point p)
{
    cv::Point2i pf(p.x/res,p.y/res);
    return pf;
}

geometry_msgs::Point Planner::convertI2W(cv::Point2i p)
{
    geometry_msgs::Point pf;
    pf.x=p.x*res;
    pf.y=p.y*res;
    return pf;
}

bool Planner::plan(void)
{
    if(map_rcv[0] && map_rcv[1])
    {
        return true;
    }
    return false;
}

// Replacement SIGINT handler
void HandlerStop(int)
{
    ROS_INFO("Failed generation of pddl.");
    ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner", ros::init_options::NoSigintHandler);
  signal(SIGINT, HandlerStop);

  ros::NodeHandle nh("~");

  Planner planner(nh);

  ros::Rate loop_rate(10);

  bool suc=false;

  while (ros::ok())
  {
    ros::spinOnce();
    if(planner.plan())
    {
        suc=true;
        break;
    }
    loop_rate.sleep();
  }

  if(!suc)
      ROS_INFO("Failed generation of pddl.");
  else
      ROS_INFO("Successful generation of pddl.");

  ros::shutdown();

  return 0;
}
