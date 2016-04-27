#include "ros/ros.h"
#include "ros/package.h"
#include <signal.h>
#include "nav_msgs/OccupancyGrid.h"
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <opencv2/core/core.hpp>
#include "ray.hpp"
#include "std_srvs/Empty.h"

using namespace std;

class MapGen{
private:
    ros::NodeHandle nh_;
    ros::ServiceServer service;
    ros::Publisher pub;
    nav_msgs::OccupancyGrid grid;
    bool gen;
    float res;
    int width,height;
    cv::Mat or_map;
    cv::Point2i convertW2I(geometry_msgs::Point p);
    geometry_msgs::Point convertI2W(cv::Point2i p);
    bool process(stringstream& iss);
    bool convert(void);
    bool generate(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
public:
    MapGen(ros::NodeHandle nh): nh_(nh){
        pub=nh_.advertise<nav_msgs::OccupancyGrid>("map", 1,true);
        service = nh_.advertiseService("generateMap", &MapGen::generate, this);
        gen=false;
    }
    bool run(void);
    void publish(void);
};

bool MapGen::generate(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res){
    gen=true;
    return true;
}



cv::Point2i MapGen::convertW2I(geometry_msgs::Point p){
    cv::Point2i pf(round(p.x/res),round(p.y/res));
    return pf;
}

geometry_msgs::Point MapGen::convertI2W(cv::Point2i p){
    geometry_msgs::Point pf;
    pf.x=p.x*res+0.5*res;
    pf.y=p.y*res+0.5*res;
    return pf;
}

bool MapGen::convert(void){
    grid.header.frame_id="map";
    grid.header.stamp=ros::Time::now();
    grid.data.resize(width*height);
    std::vector<signed char>::iterator mapDataIter = grid.data.begin();
    for(int i=0;i<width;i++){
        for(int j=0;j<height;j++){
            if(or_map.at<uchar>(j,i)==255)
                *mapDataIter=0;
            else
                *mapDataIter=100;
            mapDataIter++;
        }
    }
    return true;
}



bool MapGen::process(stringstream& iss){
    if(iss>>res>>height>>width){
        if( res>0 && height>0 && width>0 ){
            grid.info.height=width;
            grid.info.map_load_time=ros::Time::now();
            grid.info.resolution=res;
            grid.info.width=height;
            float theta;
            if(iss>>grid.info.origin.position.x>>grid.info.origin.position.y>>theta){
                grid.info.origin.orientation.x=0;
                grid.info.origin.orientation.y=0;
                grid.info.origin.orientation.z=sin(theta/2);
                grid.info.origin.orientation.w=cos(theta/2);
                geometry_msgs::Point p0, p1;
                cv::Point2i p0i, p1i;
                or_map=cv::Mat::ones(height, width, CV_8UC1)*255;
                while(iss>>p0.x>>p0.y>>p1.x>>p1.y){
                    p0i=convertW2I(p0);
                    p1i=convertW2I(p1);
                    float dist_t=sqrt( (p0i.x-p1i.x)*(p0i.x-p1i.x)+(p0i.y-p1i.y)*(p0i.y-p1i.y) );
                    //cout<<p0.x<<" "<<p0.y<<" "<<p1.x<<" "<<p1.y<<" "<<dist_t<<endl;
                    raytracing(&or_map, p0i, p0i, p1i, dist_t);
                    if(p0i.x>=0 && p0i.x<height && p0i.y>=0 && p0i.y<width)
                        or_map.at<uchar>(p0i.x,p0i.y)=0;
                }
                //cout<<res<<" "<<height<<" "<<width<<" "<<grid.info.origin.position.x<<" "<<grid.info.origin.position.y<<" "<<grid.info.origin.orientation.w<<endl;
                //cv::imshow("TestSQwA",or_map);
                //cv::waitKey(10);
                //cout<<(int)or_map.at<uchar>(50,50)<<" "<<(int)or_map.at<uchar>(50,51)<<" "<<(int)or_map.at<uchar>(51,55)<<" "<<(int)or_map.at<uchar>(49,55)<<endl;
                return true;
            }
        }
    }
    return false;
}

bool MapGen::run(void){
    if(gen){
        ifstream myfile;
        string mapFile = ros::package::getPath("map_transform").append("/maps/map.txt");
        myfile.open(mapFile);
        if (myfile.is_open()){
            stringstream ss;
            string line;
            while (getline(myfile, line)) {
                //input.push_back(line);
                ss<<line<<endl;
            }
            if(process(ss))
                if(convert())
                    return true;
        }
        else{
            //ROS_INFO("Failed to open Map file.");
        }
        myfile.close();
    }
    return false;
}

bool suc=false;

void MapGen::publish(void){
    if(suc){
        pub.publish(grid);
    }
}

// Replacement SIGINT handler
void HandlerStop(int){
    if(!suc)
        ROS_INFO("Failed to open Map file.");
    ros::shutdown();
    exit(-1);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "map", ros::init_options::NoSigintHandler);
  signal(SIGINT, HandlerStop);
  ros::NodeHandle nh;//("~");
  MapGen map(nh);
  ros::Rate loop_rate(10);
  while (ros::ok()){
    ros::spinOnce();
    if(!suc){
        if(map.run()){
            suc=true;
            ROS_INFO("Successfully read Map file.");
        }
    }
    map.publish();
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}
