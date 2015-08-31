#ifndef CLUSTERING_HPP
#define CLUSTERING_HPP

#include <opencv2/core/core.hpp>

template <typename T>
class Cluster {
public:
    std::vector<T> frontier;
    std::vector<T> rest;

    Cluster();
    void append(Cluster<T> b);
    void print(void);
};

template <typename T>
Cluster<T> clustering(Cluster<T>& clust,typename std::vector<T>::iterator, int num=0);

template <typename T>
std::vector<std::vector<T> > cluster_points(std::vector<T> frontiers, int num=0);

template <typename T>
std::vector<T> cluster_points(std::vector<T>& frontiers, typename std::vector<T>::iterator,  int num=0);

std::vector<cv::Mat> cluster_points(std::vector<cv::Mat> points, cv::Point3i pos);

class ClusterLists {
public:
    std::vector<cv::Point> cluster;
    std::vector<cv::Point> extremes;
    cv::Mat img;
    cv::Mat rest;
    ClusterLists();
};

ClusterLists cluster_points(cv::Mat orig, cv::Point pos);

std::vector<ClusterLists> cluster_points(cv::Mat orig);

ClusterLists cluster_points(std::vector<cv::Point> points, cv::Point size, cv::Point pos);

std::vector<ClusterLists> cluster_points(std::vector<cv::Point> points, cv::Point size);

std::vector<cv::Point> extremes_cluster(std::vector<cv::Point> points);

#endif // CLUSTERING_HPP
