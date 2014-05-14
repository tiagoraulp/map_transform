#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

#include <sstream>


class Reach_transf{
private:
    ros::NodeHandle nh_;

    ros::Publisher pub;
    ros::Publisher pub2;
    ros::Publisher pub3;
    
    ros::Subscriber sub;


    int count;

    std::vector<std::vector<signed char> > n_map;
    std::vector<std::vector<signed char> > f_map;
    std::vector<std::vector<signed char> > fr_map;

    void transf(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    std::vector<std::vector<signed char> > close (std::vector<std::vector<signed char> > map,float radius);

public:

    Reach_transf(ros::NodeHandle nh): nh_(nh)
    {

        pub = nh_.advertise<nav_msgs::OccupancyGrid>("n_map", 1,true);
        pub2 = nh_.advertise<nav_msgs::OccupancyGrid>("f_map", 1,true);
        pub3 = nh_.advertise<nav_msgs::OccupancyGrid>("fr_map", 1,true);

        sub = nh_.subscribe("map", 1, &Reach_transf::transf, this);

        count=0;

    }
};


std::vector<std::vector<signed char> > Reach_transf::close (std::vector<std::vector<signed char> > map,float radius)
{
    //std::vector<std::vector<signed char> >
            n_map=std::vector<std::vector<signed char> >(map);

    int rad=ceil(radius);

    for(int i=0;i<map.size();i++){
        for(int j=0;j<map[i].size();j++){

            for (int k=-rad;k<=rad;k++){
                 for (int l=-rad;l<=rad;l++){

                     if( (map[std::min(std::max(i+k,0),(int)map.size()-1)][std::min(std::max(j+l,0),(int)map[i].size()-1)]==1) && (k*k+l*l<=radius*radius) )
                        n_map[i][j]=1;

                }
            }
        }
    }

    //std::vector<std::vector<signed char> > f_map(n_map);
    f_map=std::vector<std::vector<signed char> >(n_map);

    for(int i=0;i<n_map.size();i++){
        for(int j=0;j<n_map[i].size();j++){

            for (int k=-rad;k<=rad;k++){
                 for (int l=-rad;l<=rad;l++){

                     if( (n_map[std::min(std::max(i+k,0),(int)n_map.size()-1)][std::min(std::max(j+l,0),(int)n_map[i].size()-1)]!=1) && (k*k+l*l<=radius*radius) )
                     {
                         if (map[i][j]==-1)
                             f_map[i][j]=-1;
                         else
                             f_map[i][j]=0;
                     }

                }
            }
        }
    }

    fr_map=std::vector<std::vector<signed char> >(f_map);

    std::vector<std::vector<char> > labels(0);

    for(int i=0;i<fr_map.size();i++){
        for(int j=0;j<fr_map[i].size();j++){

            if(fr_map[i][j]!=1) {

                for (int k=-1;k<=0;k++){
                     for (int l=-1;l<=-2*k-1;l++){

                         if( (i+k)>=0 && (i+k)<fr_map.size() && (j+l)>=0 && (j+l)<fr_map[i].size() )
                         {

                             if( (fr_map[i+k][j+l]>1))
                             {
                                 if (fr_map[i][j]<1)
                                     fr_map[i][j]=fr_map[i+k][j+l];
                                 else if (fr_map[i+k][j+l]!=fr_map[i][j])
                                 {
                                     //f_map[i][j]=0;
                                 }
                             }
                         }
                    }
                }

                if (fr_map[i][j]<1)
                {
                    fr_map[i][j]=2+labels.size();
                    //labels
                }

            }
        }
    }


    return fr_map;
}

void Reach_transf::transf(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]", msg->header.seq);


    /////////////////////////////////
    std::vector<std::vector<signed char> > GridMap(msg->info.height, std::vector<signed char>(msg->info.width, -1));


    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    signed char map_occ_thres = 90;
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){           
                signed char val;
                if(*mapDataIterC >  map_occ_thres)
                  val = 1;
                else if(*mapDataIterC == 0)
                  val = 0;
                else
                  val = -1;
                GridMap[i][j]=val;

                mapDataIterC++;
            }
    }

    GridMap=close(GridMap,3);

    ////

    nav_msgs::OccupancyGrid p_msg;

    p_msg.data.resize(msg->data.size(),-1);
    std::vector<signed char>::iterator mapDataIter = p_msg.data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
                signed char val;
                if(GridMap[i][j] >= 1)
                    val = 100;
                else if(GridMap[i][j] == 0)
                    val = 0;
                else
                    val = -1;
                *mapDataIter=val;

                mapDataIter++;
            }
    }


    //////////////////////////////////

    p_msg.header.stamp = ros::Time::now();
    p_msg.header.frame_id = msg->header.frame_id;
    p_msg.info=msg->info;
    std::stringstream ss;
    ss << "Message sent " << count;
    //msg.data = ss.str();

    ROS_INFO("%s", ss.str().c_str());


    p_msg.info.map_load_time=ros::Time::now();
    pub.publish(p_msg);

    mapDataIter = p_msg.data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
                signed char val;
                if(n_map[i][j] >= 1)
                    val = 100;
                else if(n_map[i][j] == 0)
                    val = 0;
                else
                    val = -1;
                *mapDataIter=val;

                mapDataIter++;
            }
    }

    pub2.publish(p_msg);

    ++count;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "reach");
  ros::NodeHandle nh;

  Reach_transf reach(nh);

  //ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("n_map", 1);

  //ros::Subscriber sub = nh.subscribe("map", 1, &Reach_transf::transf, &reach);

  ros::Rate loop_rate(2);

  
  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
