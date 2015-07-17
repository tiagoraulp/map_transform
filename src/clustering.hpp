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


#endif // CLUSTERING_HPP
