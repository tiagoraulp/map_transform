#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>
#include <opencv2/core/core.hpp>
#include "map_transform/ObsList.h"
#include "map_transform/ObsNumber.h"

#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>


#include <dynamic_reconfigure/server.h>
#include <map_transform/ImportanceMapPlannerConfig.h>

using namespace std;

class MapPlanner{
private:
    vector<float > probs;
    vector<vector<float> > attemps;
    vector<cv::Mat> maps, maps_obs;
    vector<unsigned short> number_obs;
    int obs_debug;
    vector<vector<vector<float> > > importance;
    ros::NodeHandle nh_;
    ros::Publisher pub, pub_imp;
    ros::Subscriber sub;
    ros::ServiceClient client;
    nav_msgs::OccupancyGrid grid, final, importance_map;
    bool suc, finished, rcv, processed;
    float res;
    int width,height;
    int n_obs, n_iter, curr_iter;
    cv::Mat or_map, final_map, map_temp;
    dynamic_reconfigure::Server<map_transform::ImportanceMapPlannerConfig> server;
    dynamic_reconfigure::Server<map_transform::ImportanceMapPlannerConfig>::CallbackType f;
    void rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    bool convertCV2Map(cv::Mat map);
    void convertVM2Map(unsigned int obs);
    void processImportances(void);
    void configCallback(map_transform::ImportanceMapPlannerConfig &config, uint32_t level);
public:
    MapPlanner(ros::NodeHandle nh): nh_(nh){
        pub=nh_.advertise<nav_msgs::OccupancyGrid>("prob_map", 1,true);
        pub_imp=nh_.advertise<nav_msgs::OccupancyGrid>("importance_map", 1,true);
        sub = nh_.subscribe("map", 1, &MapPlanner::rcv_map, this);
        suc=true;
        rcv=false;
        finished=false;
        processed=false;
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
        n_iter=4*n_obs*2;
        attemps.resize(n_iter);
        maps.resize(n_iter);
        maps_obs.resize(n_obs*2);
        number_obs.resize(n_obs*2);
        curr_iter=0;
        srand (time(NULL));
        obs_debug=0;

        f = boost::bind(&MapPlanner::configCallback, this, _1, _2);
        server.setCallback(f);

    }
    void run(void);
    void publish(void);
};

void MapPlanner::configCallback(map_transform::ImportanceMapPlannerConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d",
            config.obs_index);
  obs_debug=config.obs_index;
}

void MapPlanner::rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    or_map = cv::Mat(msg->info.width, msg->info.height, CV_32FC1);
    map_temp = cv::Mat(msg->info.width, msg->info.height, CV_8UC1);

    grid=*msg;

    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
            if(*mapDataIterC < 50){
                or_map.at<float>(j,i) = 255;
                map_temp.at<uchar>(j,i) = 1;
            }
            else{
                or_map.at<float>(j,i) = 0;
                map_temp.at<uchar>(j,i) = 0;
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
    for(int i=0;i<height;i++){
        for(int j=0;j<width;j++){
           *mapDataIter=100-(signed char)(map.at<float>(j,i)/255*100);
           mapDataIter++;
        }
    }
    return true;
}

void MapPlanner::convertVM2Map(unsigned int obs){
    if((int)obs>=n_obs)
        obs=n_obs-1;
    importance_map.header.frame_id="map";
    importance_map.header.stamp=ros::Time::now();
    importance_map.data.resize(width*height);
    std::vector<signed char>::iterator mapDataIter = importance_map.data.begin();
    for(int i=0;i<height;i++){
        for(int j=0;j<width;j++){
           *mapDataIter=100-(signed char)(importance[j][i][obs]*100.0);
           mapDataIter++;
        }
    }
}

void MapPlanner::run(void){

    if(suc && rcv){
        //cout<<"Part 1: Current: "<<curr_iter<<endl;
        rcv=false;
        if(curr_iter==1){
            final_map=or_map.clone();
        }
        else{
            final_map=((float)curr_iter)/((float)curr_iter+1.0)*final_map+(1)/((float)curr_iter+1.0)*or_map;
        }
        maps[curr_iter-1]=map_temp.clone();
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
            //cout<<"Part 2: Current: "<<curr_iter<<endl;
            map_transform::ObsList srv;
            srv.request.obs.resize(n_obs);
            attemps[curr_iter].resize(n_obs);
            for(int i=0; i<n_obs;i++){
                srv.request.obs[i]=(((float)rand())/((float)RAND_MAX))<probs[i]?1:0;
                attemps[curr_iter][i]=srv.request.obs[i];
            }
            client.call(srv);
            if(srv.response.result){
                rcv=false;
                suc=false;
                curr_iter++;
            }
        }
    }
    if(finished && !processed){
        processImportances();
        processed=true;
        importance_map=grid;
        cout<<"Completed Analysis! Yeah!"<<endl;
    }
    if(processed){
        convertVM2Map(obs_debug);
    }
}

void MapPlanner::processImportances(void){
    importance.assign(width, vector<vector<float> >(height, vector<float>(n_obs, 0)));
    for(unsigned int i=0;i<maps_obs.size();i++){
        maps_obs[i] = cv::Mat::zeros(width, height, CV_16UC1);
    }
    number_obs.assign(maps_obs.size(), 0);
    for(unsigned int i=0; i<maps.size(); i++){
        for(unsigned int oo=0; oo<attemps[i].size(); oo++){
            if(attemps[i][oo]==0){
                number_obs[oo*2]++;
            }
            else{
                number_obs[oo*2+1]++;
            }
            for(int rr=0; rr<maps[i].rows;rr++){
                for(int cc=0; cc<maps[i].cols;cc++){
                    if(attemps[i][oo]==0){
                        maps_obs[oo*2].at<unsigned short>(rr,cc)+=(unsigned short)maps[i].at<uchar>(rr,cc);
                    }
                    else{
                        maps_obs[oo*2+1].at<unsigned short>(rr,cc)+=(unsigned short)maps[i].at<uchar>(rr,cc);
                    }
                }
            }
        }
    }
    float value_noObs, value_withObs, diff, diffTotal;
    vector<float> diffs(n_obs, 0);
    for(int rr=0; rr<width;rr++){
        for(int cc=0; cc<height;cc++){
            diffTotal=0;
            for(unsigned int oo=0; oo<(maps_obs.size()-1);oo=oo+2){
                ///TODO: what if number_obs is zero?
                value_noObs=1.0-((float)maps_obs[oo].at<unsigned short>(rr,cc))/((float)number_obs[oo]);
                value_withObs=1.0-((float)maps_obs[oo+1].at<unsigned short>(rr,cc))/((float)number_obs[oo+1]);
                diff=value_withObs-value_noObs;
                if(diff<0)
                    diff=0;
                diffs[oo/2]=diff;
                diffTotal+=diff;
                if(rr==20 && cc==100 ){
                    cout<<"Current "<<oo/2<<" out of "<<maps_obs.size()/2<<": "<<value_noObs<<" "<<value_withObs<<" "<<diffs[oo/2]<<"; "<<number_obs[oo]<<" "<<number_obs[oo+1]<<endl;
                }
            }
            //////// TODO:
            /// Correr visibility com e sem obstaculo, com outros todos abertos e depois fechados, para detetar regiões de interesse
            /// para a importância?...
            float test=0;
            for(unsigned int oo=0; oo<diffs.size();oo++){
                if(diffTotal==0)
                    diffs[oo]=0;
                else
                    diffs[oo]/=diffTotal;
                test+=diffs[oo];
                importance[rr][cc][oo]=diffs[oo];
                if(rr==20 && cc==100 ){
                    cout<<"Current "<<oo<<" out of "<<diffs.size()<<": "<<diffs[oo]<<" "<<diffTotal<<endl;
                }
            }
            if(rr==20 && cc==100 ){
                cout<<test<<endl;
            }
        }
    }
}

void MapPlanner::publish(void){
    if(finished){
        pub.publish(final);
    }
    if(processed){
        pub_imp.publish(importance_map);
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
