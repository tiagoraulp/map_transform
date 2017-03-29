#include "ros/ros.h"
#include "ros/package.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <opencv2/core/core.hpp>
#include "ray.hpp"
#include "std_srvs/Empty.h"
#include "map_transform/ObsList.h"
#include "map_transform/ObsNumber.h"

using namespace std;

class ObsGen{
private:
    vector<vector<geometry_msgs::Point> > obstacles;
    vector<float> probs;
    ros::NodeHandle nh_;
    ros::ServiceServer service, service2;
    ros::Publisher pub, pubObs;
    ros::Subscriber sub;
    nav_msgs::OccupancyGrid grid, gridWithObs;
    bool gen, suc, suc_obs, loaded;
    float res;
    int width,height;
    cv::Mat or_map, obs_map;
    cv::Point2i convertW2I(geometry_msgs::Point p);
    geometry_msgs::Point convertI2W(cv::Point2i p);
    nav_msgs::OccupancyGrid processObs(vector<unsigned char> obs);
    bool save(stringstream& iss);
    bool convertCV2Map(cv::Mat map);
    void printObs(void);
    bool generate(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
    bool generateObsMap(map_transform::ObsList::Request  &req, map_transform::ObsList::Response &res);
    bool getObsNumber(map_transform::ObsNumber::Request  &req, map_transform::ObsNumber::Response &res);
    void rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg);
public:
    ObsGen(ros::NodeHandle nh): nh_(nh){
        pub=nh_.advertise<nav_msgs::OccupancyGrid>("original", 1,true);
        pubObs=nh_.advertise<nav_msgs::OccupancyGrid>("obstacle_map", 1,true);
        service = nh_.advertiseService("generateObsMap", &ObsGen::generateObsMap, this);
        service2 = nh_.advertiseService("getObsNumber", &ObsGen::getObsNumber , this);
        sub = nh_.subscribe("/map", 1, &ObsGen::rcv_map, this);
        gen=false;
        suc=false;
        suc_obs=false;
        loaded=false;
    }
    bool run(void);
    bool load(void);
    void publish(void);
};


void ObsGen::rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    //cout<<"Mapppppppp"<<endl;
    or_map = cv::Mat(msg->info.width, msg->info.height, CV_8UC1);

    grid=*msg;

    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
            if(*mapDataIterC < 50){
                or_map.at<uchar>(j,i) = 255;
            }
            else{
                or_map.at<uchar>(j,i) = 0;
            }
            mapDataIterC++;
        }
    }
    res=msg->info.resolution;
    width=msg->info.width;
    height=msg->info.height;
    suc=true;
}

bool ObsGen::generate(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res){
    //gen=true;
    return true;
}

nav_msgs::OccupancyGrid ObsGen::processObs(vector<unsigned char> obs){
    cv::Point2i p0i, p1i;
    cv::Mat temp=or_map.clone();
    for(int i=0; i<obs.size();i++){
        if(!obs[i] || i>=obstacles.size())
            continue;
        for(int j=0; j<obstacles[i].size();j++){
            p0i=convertW2I(obstacles[i][j]);
            //cout<<p0i.x<<" "<<p0i.y<<endl;
            if( (j+1)>=obstacles[i].size() ){
                continue;
            }
            p1i=convertW2I(obstacles[i][j+1]);

            float dist_t=sqrt( (p0i.x-p1i.x)*(p0i.x-p1i.x)+(p0i.y-p1i.y)*(p0i.y-p1i.y) );
            //cout<<p0i.x<<" "<<p0i.y<<" "<<p1i.x<<" "<<p1i.y<<" "<<dist_t<<endl;
            raytracing(&temp, p0i, p0i, p1i, dist_t);
            //cout<<"Raytracing"<<endl;
            if(p0i.x>=0 && p0i.x<temp.rows && p0i.y>=0 && p0i.y<temp.cols)
                temp.at<uchar>(p0i.x,p0i.y)=0;
        }
    }
    convertCV2Map(temp);
    suc_obs=true;
    return gridWithObs;
}

bool ObsGen::generateObsMap(map_transform::ObsList::Request  &req, map_transform::ObsList::Response &res){
    //cout<<"Received Service Request "<<suc<<endl;
    if(suc){
        //cout<<"Testtttttt"<<endl;
        gridWithObs=grid;
        res.map=processObs(req.obs);
        res.result=true;
        gen=true;
    }
    else{
        res.map=grid;
        res.result=false;
    }
    return true;
}

bool ObsGen::getObsNumber(map_transform::ObsNumber::Request  &req, map_transform::ObsNumber::Response &res){
    //cout<<"Received Service Request "<<suc<<endl;
    while(!loaded){
        continue;
    }
    //res.number.data=obstacles.size();
    res.probs.resize(probs.size());
    for(int i=0;i<probs.size();i++)
        res.probs[i].data=probs[i];
    return true;
}

cv::Point2i ObsGen::convertW2I(geometry_msgs::Point p){
    cv::Point2i pf(round(p.x/res),round(p.y/res));
    return pf;
}

geometry_msgs::Point ObsGen::convertI2W(cv::Point2i p){
    geometry_msgs::Point pf;
    pf.x=p.x*res+0.5*res;
    pf.y=p.y*res+0.5*res;
    return pf;
}

bool ObsGen::convertCV2Map(cv::Mat map){
    gridWithObs.header.frame_id="map";
    gridWithObs.header.stamp=ros::Time::now();
    gridWithObs.data.resize(width*height);
    std::vector<signed char>::iterator mapDataIter = gridWithObs.data.begin();
    for(int i=0;i<width;i++){
        for(int j=0;j<height;j++){
            if(map.at<uchar>(j,i)==255)
                *mapDataIter=0;
            else
                *mapDataIter=100;
            mapDataIter++;
        }
    }
    return true;
}

bool ObsGen::save(stringstream& iss){
    obstacles.clear();
    probs.clear();
    geometry_msgs::Point p0;
    string line;
    bool obs=true;
    while (getline(iss, line)) {
        vector<geometry_msgs::Point> temp;
        stringstream ss(line);
        float prob;
        if(obs){
            while(ss>>p0.x>>p0.y){
                temp.push_back(p0);
            }
            obstacles.push_back(temp);
            obs=!obs;
        }
        else{
            ss>>prob;
            probs.push_back(prob);
            obs=!obs;
        }
    }
    return true;
}

void ObsGen::printObs(void){
    for(unsigned int i=0; i<obstacles.size();i++){
        cout<<"Obstacle "<<i+1<<" points: ";
        for(unsigned int j=0; j<obstacles[i].size();j++){
            cout<<"("<<obstacles[i][j].x<<", "<<obstacles[i][j].y<<"); ";
        }
        cout<<" Probability: "<<probs[i]<<endl;
    }
}

bool ObsGen::load(void){
    ifstream myfile;
    string mapFile = ros::package::getPath("map_transform").append("/maps/map_obs.txt");
    myfile.open(mapFile);
    if (myfile.is_open()){
        stringstream ss;
        string line;
        while (getline(myfile, line)) {
            //input.push_back(line);
            ss<<line<<endl;
        }
        myfile.close();
        if(save(ss)){
            //if(convertCV2Map())
            printObs();
            loaded=true;
            return true;
        }
    }
    myfile.close();
    return false;
}

bool ObsGen::run(void){
    return true;
}

void ObsGen::publish(void){
    if(suc){
        pub.publish(grid);
    }
    if(suc_obs && gen){
        pubObs.publish(gridWithObs);
        gen=false;
    }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "obsmap");
  ros::NodeHandle nh;//("~")
  ObsGen map(nh);
  ros::Rate loop_rate(50);
  map.load();
  while (ros::ok()){
    ros::spinOnce();
    map.publish();
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}
