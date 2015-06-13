#ifndef UNREACHABLE_HPP
#define UNREACHABLE_HPP

#include <vector>
#include <opencv2/core/core.hpp>
#include "labelling.hpp"
#include "clustering.hpp"

using namespace std;

class Unreachable
{
    vector<vector<cv::Point> > labels_unreach;
    cv::Mat act;
    void getRegions(cv::Mat map_or, cv::Mat act_map);
public:
    cv::Mat regions;
    vector<vector<vector<cv::Point> > > frontiers;
    cv::Mat unreach_map;

    Unreachable(cv::Mat map_or, cv::Mat act_map);
    void getFrontiers(void);
};

#endif // UNREACHABLE_HPP
