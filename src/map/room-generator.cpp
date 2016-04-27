#include "ros/ros.h"
#include "ros/package.h"
#include <signal.h>
#include <fstream>
#include <opencv2/core/core.hpp>
#include "std_srvs/Empty.h"

using namespace std;

class RoomGen{
private:
    ros::NodeHandle nh_;
    float res;
    int width,height;
    void process(ofstream& map);
    ros::ServiceClient permission;
public:
    RoomGen(ros::NodeHandle nh): nh_(nh){
        srand (time(NULL));
        permission = nh_.serviceClient<std_srvs::Empty>("generateMap");
    }
    bool run(void);
    bool finish(void);
};

void RoomGen::process(ofstream& map){
    res=0.1;
    height=100;
    width=200;
    map<<res<<" "<<height<<" "<<width<<" "<<endl;
    map<<0<<" "<<0<<" "<<0<<" "<<endl;
    cv::Point2f p0, p1, pdn, pdp;
    p0.x=5;p0.y=5;p1=p0; pdn=p0; pdp=p0;
    float wallMin=2.0, wallMax=8.0;
    float wall=wallMin+((float)rand()/RAND_MAX)*(wallMax-wallMin);
    p1.y+=(int)round(wall);

    float doorMin=0.6, doorMax=2.0;
    float door_pos=doorMin/2.0+((float)rand()/RAND_MAX)*(wall-2.0*doorMin/2.0);
    float doorSize=doorMin+((float)rand()/RAND_MAX)*(doorMax-doorMin);
    float doorN=door_pos-doorSize/2.0, doorP=door_pos+doorSize/2.0;

    if(doorN<0)
        doorN=0;
    if(doorP>wall)
        doorP=wall;

    door_pos=(doorN+doorP)/2.0;
    doorSize=(doorP-doorN)/2.0;

    pdn.y+=doorN;
    pdp.y+=doorP;

    map<<p0.x<<" "<<p0.y<<" "<<pdn.x<<" "<<pdn.y<<endl;
    map<<pdp.x<<" "<<pdp.y<<" "<<p1.x<<" "<<p1.y<<endl;


    //cout<<p0.x<<" "<<p0.y<<" "<<p1.x<<" "<<p1.y<<" "<<dist_t<<endl;
    //cout<<res<<" "<<height<<" "<<width<<" "<<grid.info.origin.position.x<<" "<<grid.info.origin.position.y<<" "<<grid.info.origin.orientation.w<<endl;
    //cv::imshow("TestSQwA",or_map);
    //cv::waitKey(10);
    //cout<<(int)or_map.at<uchar>(50,50)<<" "<<(int)or_map.at<uchar>(50,51)<<" "<<(int)or_map.at<uchar>(51,55)<<" "<<(int)or_map.at<uchar>(49,55)<<endl;
}

bool RoomGen::run(void){
    ofstream myfile;
    string mapFile = ros::package::getPath("map_transform").append("/maps/map.txt");
    myfile.open(mapFile);
    if (myfile.is_open()){
        process(myfile);
        return true;
    }
    else{
        //ROS_INFO("Failed to open Map file.");
    }
    myfile.close();
    return false;
}

bool RoomGen::finish(void){
    std_srvs::Empty srv;
    return permission.call(srv);
}

bool suc=false;

// Replacement SIGINT handler
void HandlerStop(int){
    if(!suc)
        ROS_INFO("Failed to generate Map file.");
    ros::shutdown();
    exit(-1);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "rooms", ros::init_options::NoSigintHandler);
  signal(SIGINT, HandlerStop);
  ros::NodeHandle nh;//("~");
  RoomGen rooms(nh);
  ros::Rate loop_rate(10);
  while (ros::ok()){
    ros::spinOnce();
    if(!suc){
        if(rooms.run()){
            suc=true;
            ROS_INFO("Successfully generated Map file.");
        }
    }
    else{
        if (rooms.finish()){
            ROS_INFO("Successful request for Map Generation");
            break;
        }
        else{
            ROS_INFO("Failed to request Map Generation");
        }
    }
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}
