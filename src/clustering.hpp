#ifndef CLUSTERING_HPP
#define CLUSTERING_HPP

#include <vector>
#include <opencv2/core/core.hpp>
#include <iostream>

using namespace std;

class Cluster {
public:
    vector<cv::Point> frontier;
    vector<cv::Point> rest;

    Cluster();
    void append(Cluster b);
    void print(void);
};

Cluster clustering(Cluster clust, unsigned int index);

vector<vector<cv::Point> > cluster_points(vector<cv::Point> frontiers);


#endif // CLUSTERING_HPP
