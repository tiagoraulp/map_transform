#ifndef CRITPOINTS_HPP
#define CRITPOINTS_HPP

#include <vector>
#include <opencv2/core/core.hpp>
#include "vector_utils.hpp"
#include "obtuseAngle.hpp"

extern const double PI;

using namespace std;

class CritPoints
{
private:
    cv::Mat r_map, map_or;
    int infl;
    cv::Point2i critP;
    vector<cv::Point2i> frontier;
    vector<float> extremes;
    vector<cv::Point2i> extremesP;
    unsigned int obt;
    cv::Point2i find_extreme(cv::Point2i pt, float a0, float a1, bool special=false);
    void extremePoints(cv::Point2i pt, cv::Point2i pt2, float a0, float a1, bool first=false);
    float getAngle(cv::Point2i pt);
public:
    CritPoints(cv::Mat map, cv::Mat reach, int rs);
    cv::Point2i find_crit_point(vector<cv::Point> frontier);
    vector<float> frontier_extremes(void);
    cv::Point2i getCrit(void);
    vector<float> getExtremes(void);
    vector<cv::Point2i> getExtremesP(void);
    unsigned int getObt(void);
};

#endif // CRITPOINTS_HPP
