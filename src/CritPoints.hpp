#ifndef CRITPOINTS_HPP
#define CRITPOINTS_HPP

#include <opencv2/core/core.hpp>


class CritPoints
{
private:
    cv::Mat r_map, map_or;
    int infl;
    cv::Point2i critP;
    std::vector<cv::Point2i> frontier;
    std::vector<float> extremes;
    std::vector<cv::Point2i> extremesP;
    unsigned int obt;
    cv::Point2i find_extreme(cv::Point2i pt, float a0, float a1, bool special=false);
    void extremePoints(cv::Point2i pt, cv::Point2i pt2, float a0, float a1, bool first=false);
    float getAngle(cv::Point2i pt);
public:
    CritPoints(cv::Mat map, cv::Mat reach, int rs);
    cv::Point2i find_crit_point(std::vector<cv::Point> frontier);
    std::vector<float> frontier_extremes(void);
    cv::Point2i getCrit(void);
    std::vector<float> getExtremes(void);
    std::vector<cv::Point2i> getExtremesP(void);
    unsigned int getObt(void);
};

#endif // CRITPOINTS_HPP