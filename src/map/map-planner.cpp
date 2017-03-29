#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>
#include <opencv2/core/core.hpp>
#include "map_transform/ObsList.h"
#include "map_transform/ObsNumber.h"

#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>

using namespace std;

class MapPlanner{
private:
    vector<float > probs;
    ros::NodeHandle nh_;
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::ServiceClient client;
    nav_msgs::OccupancyGrid grid, final;
    bool suc, finished, rcv;
    float res;
    int width,height;
    int n_obs, n_iter, curr_iter;
    cv::Mat or_map, final_map;
    void rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    bool convertCV2Map(cv::Mat map);
public:
    MapPlanner(ros::NodeHandle nh): nh_(nh){
        pub=nh_.advertise<nav_msgs::OccupancyGrid>("prob_map", 1,true);
        sub = nh_.subscribe("map", 1, &MapPlanner::rcv_map, this);
        suc=true;
        rcv=false;
        finished=false;
        ros::service::waitForService("generateObsMap");
        cout<<"Found Map Service"<<endl;
        client = nh_.serviceClient<map_transform::ObsList>("generateObsMap");

        ros::service::waitForService("getObsNumber");
        cout<<"Found Service ObsNumber"<<endl;
        ros::ServiceClient client2 = nh_.serviceClient<map_transform::ObsNumber>("getObsNumber");
        map_transform::ObsNumber srv;
        client2.call(srv);
        n_obs=srv.response.probs.size();
        probs.resize(n_obs);
        for(int i=0;i<n_obs;i++)
            probs[i]=srv.response.probs[i].data;


        cout<<"Got ObsNumber: "<<n_obs<<endl;
        n_iter=4*n_obs;
        curr_iter=0;
        srand (time(NULL));
    }
    void run(void);
    void publish(void);
};


void MapPlanner::rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    or_map = cv::Mat(msg->info.width, msg->info.height, CV_32FC1);

    grid=*msg;

    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
            if(*mapDataIterC < 50){
                or_map.at<float>(j,i) = 255;
            }
            else{
                or_map.at<float>(j,i) = 0;
            }
            mapDataIterC++;
        }
    }
    res=msg->info.resolution;
    width=msg->info.width;
    height=msg->info.height;
    suc=true;
    rcv=true;
}


//bool MapPlanner::generateObsMap(map_transform::ObsList::Request  &req, map_transform::ObsList::Response &res){

bool MapPlanner::convertCV2Map(cv::Mat map){
    final.header.frame_id="map";
    final.header.stamp=ros::Time::now();
    final.data.resize(width*height);
    std::vector<signed char>::iterator mapDataIter = final.data.begin();
    for(int i=0;i<width;i++){
        for(int j=0;j<height;j++){
           *mapDataIter=100-(signed char)(map.at<float>(j,i)/255*100);
           mapDataIter++;
        }
    }
    return true;
}



void MapPlanner::run(void){

    if(suc && rcv){
        rcv=false;
        if(curr_iter==1){
            final_map=or_map;
        }
        else{
            final_map=((float)curr_iter)/((float)curr_iter+1.0)*final_map+(1)/((float)curr_iter+1.0)*or_map;
        }
        if(curr_iter>=n_iter){
            final=grid;
            convertCV2Map(final_map);
            finished=true;
            cout<<"Finished! :)"<<endl;
        }
    }
    //cout<<"Current: "<<curr_iter<<endl;
    if(curr_iter<n_iter){
        if(suc){
            //cout<<"Current: "<<curr_iter<<endl;
            map_transform::ObsList srv;
            srv.request.obs.resize(n_obs);
            for(int i=0; i<n_obs;i++){
                srv.request.obs[i]=(((float)rand())/((float)RAND_MAX))<probs[i]?1:0;
            }
            client.call(srv);
            if(srv.response.result){
                rcv=false;
                suc=false;
                curr_iter++;
            }

        }
    }
}

void MapPlanner::publish(void){
    if(finished){
        pub.publish(final);
    }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "mapplanner");
  ros::NodeHandle nh;//("~")
  MapPlanner map(nh);
  ros::Rate loop_rate(50);
  while (ros::ok()){
    ros::spinOnce();
    map.run();
    map.publish();
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}
