#include "unreachable.hpp"

#include "labelling.hpp"
#include "clustering.hpp"

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
    for (unsigned int k=0;k<labels_unreach.size();k++){
        cv::Mat frontiers_t=cv::Mat::zeros(unreach_map.rows, unreach_map.cols, CV_8UC1);
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

        vector<ClusterLists> clusters_c=cluster_points(frontiers_t, map, act);
        clusters.push_back(clusters_c);
        vector<vector<cv::Point> > frontiers_c;
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
