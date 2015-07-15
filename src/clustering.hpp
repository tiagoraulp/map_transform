#ifndef CLUSTERING_HPP
#define CLUSTERING_HPP

#include <opencv2/core/core.hpp>


class Cluster {
public:
    std::vector<cv::Point> frontier;
    std::vector<cv::Point> rest;

    Cluster();
    void append(Cluster b);
    void print(void);
};

Cluster clustering(Cluster clust, unsigned int index);

std::vector<std::vector<cv::Point> > cluster_points(std::vector<cv::Point> frontiers);


#endif // CLUSTERING_HPP
