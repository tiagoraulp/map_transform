#ifndef UNREACHABLE_HPP
#define UNREACHABLE_HPP

#include <opencv2/core/core.hpp>
#include "clustering.hpp"

class Unreachable
{
    std::vector<std::vector<cv::Point> > labels_unreach;
    cv::Mat act;
    void getRegions(cv::Mat map_or, cv::Mat act_map);
public:
    cv::Mat regions;
    std::vector<std::vector<std::vector<cv::Point> > > frontiers;
    std::vector<std::vector<ClusterLists> > clusters;
    cv::Mat unreach_map;

    Unreachable(cv::Mat map_or, cv::Mat act_map);
    void getFrontiers(void);
    void getFrontiers2(void);
};

#endif // UNREACHABLE_HPP
