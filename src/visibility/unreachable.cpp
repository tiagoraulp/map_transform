#include "unreachable.hpp"

#include "labelling.hpp"
#include "clustering.hpp"
#include "vector_utils.hpp"

#include "ros/console.h"

using namespace std;


Unreachable::Unreachable(cv::Mat map_or, cv::Mat act_map) {
    pixel_count.assign(0,0);
    getRegions(map_or, act_map);
}

void Unreachable::getFrontiers(void)
{
    frontiers.clear();
    for (unsigned int k=0;k<labels_unreach.size();k++){
        vector<cv::Point> frontiers_t;
        for(unsigned int j=0;j<labels_unreach[k].size();j++){
            bool stop=false;
            for(int a=labels_unreach[k][j].x-1;a<=(labels_unreach[k][j].x+1);a++){
                for(int b=labels_unreach[k][j].y-1;b<=(labels_unreach[k][j].y+1);b++){
                    if ((a>=0)&&(b>=0)&&(a<act.rows)&&(b<act.cols) )
                    {
                        int aa=a-labels_unreach[k][j].x;
                        int bb=b-labels_unreach[k][j].y;
                        if((abs(aa)+abs(bb))==1)   ///TODO: check connectivity, if 4 is enough!!!
                            if(act.at<uchar>(a,b)==255)
                            {
                                frontiers_t.push_back(cv::Point(labels_unreach[k][j].x,labels_unreach[k][j].y));
                                stop=true;
                                break;
                            }
                    }
                }
                if(stop)
                    break;
            }
        }
        vector<vector<cv::Point> > frontiers_c=cluster_points(frontiers_t);
        frontiers.push_back(frontiers_c);
    }
}

void Unreachable::getFrontiers2(void)
{
    clusters.clear();
    cv::Mat frontiers_t;
    vector<ClusterLists> clusters_c;
    vector<vector<cv::Point> > frontiers_c;
    ROS_INFO("%ld", labels_unreach.size());
    for (unsigned int k=0;k<labels_unreach.size();k++){
        frontiers_t=cv::Mat::zeros(unreach_map.rows, unreach_map.cols, CV_8UC1);
        for(unsigned int j=0;j<labels_unreach[k].size();j++){
            bool stop=false;
            for(int a=labels_unreach[k][j].x-1;a<=(labels_unreach[k][j].x+1);a++){
                for(int b=labels_unreach[k][j].y-1;b<=(labels_unreach[k][j].y+1);b++){
                    if ((a>=0)&&(b>=0)&&(a<act.rows)&&(b<act.cols) )
                    {
                        int aa=a-labels_unreach[k][j].x;
                        int bb=b-labels_unreach[k][j].y;
                        if((abs(aa)+abs(bb))==1)   ///TODO: check connectivity, if 4 is enough!!!
                            if(act.at<uchar>(a,b)==255)
                            {
                                frontiers_t.at<uchar>(labels_unreach[k][j].x,labels_unreach[k][j].y)=255;
                                stop=true;
                                break;
                            }
                    }
                }
                if(stop)
                    break;
            }
        }

        ROS_INFO("I'm Here 111111111: %ud!!!", k);



        clusters_c=cluster_points(frontiers_t, map, act);
        clusters.push_back(clusters_c);
        frontiers_c.clear();
        for(unsigned int i=0; i<clusters_c.size();i++)
        {
            frontiers_c.push_back(clusters_c[i].cluster);
        }
        frontiers.push_back(frontiers_c);
    }
}


void Unreachable::getRegions(cv::Mat map_or, cv::Mat act_map)
{
    cv::Mat temp_labelling, temp;
    bitwise_not(map_or.clone(),temp);
    unreach_map=act_map|temp;
    bitwise_not( unreach_map.clone() , temp_labelling);
    labels_unreach=label(temp_labelling/255,4);
    regions=map_or.clone()/255;

    pixel_count.assign(labels_unreach.size(),0);
    for (unsigned int i=0;i<labels_unreach.size();i++){
        for(unsigned int j=0;j<labels_unreach[i].size();j++){
               regions.at<uchar>(labels_unreach[i][j].x,labels_unreach[i][j].y)=i+2;
               pixel_count[i]++;
        }
    }

    act=act_map.clone();
    map=map_or.clone();
}

int Unreachable::checkFrontierSize(unsigned int k, unsigned int ff){
    if(k>=frontiers.size())
        return -1;
    if(ff>=frontiers[k].size())
        return -1;
    int result=0;
    cv::Mat temp=unreach_map.clone();
    vector<cv::Point>frontier=frontiers[k][ff];
    for(unsigned int i=0; i<frontier.size();i++){
        temp.at<uchar>(frontier[i].x,frontier[i].y)=255;
    }
    for(unsigned int fr=0; fr<frontier.size();fr++){
        int lx=boundPos(frontier[fr].x-1,temp.rows);
        int ux=boundPos(frontier[fr].x+1,temp.rows);
        int ly=boundPos(frontier[fr].y-1,temp.cols);
        int uy=boundPos(frontier[fr].y+1,temp.cols);
        bool stop=false;
        for(int i=lx; i<=ux; i++){
            for(int j=ly; j<=uy; j++){
                if( temp.at<uchar>(i,j)==0 ){
                    if( max( abs(frontier[fr].x-i),abs(frontier[fr].y-j) )==1 ){
                        stop=true;
                        break;
                    }
                }
            }
            if(stop)
                break;
        }
        if(stop){
            result++;
        }
    }
    return result;
}
