#ifndef OBS_EXTREMES_HPP
#define OBS_EXTREMES_HPP

#include <vector>
#include <opencv2/core/core.hpp>
#include "chain.hpp"

using namespace std;

extern const double PI;

class ExtremesObst2Point
{
private:
    cv::Point2i critP;
    vector<Chain> ch;
    vector<float> diffAngle;
    float prev_diff;
    vector<cv::Point> ext;
    void chainAngleDiff(void);
    void firstLoop(void);
    void secLoop(void);
public:
    ExtremesObst2Point(cv::Point2i crit, vector<Chain> c);
    vector<cv::Point2i> getExt(void);
};

#endif // OBS_EXTREMES_HPP
