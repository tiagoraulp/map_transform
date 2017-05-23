#ifndef OBS_EXTREMES_HPP
#define OBS_EXTREMES_HPP

#include <opencv2/core/core.hpp>
#include "chain.hpp"

class ExtremesObst2Point
{
private:
    cv::Point2i critP;
    std::vector<Chain> ch;
    std::vector<float> diffAngle;
    float prev_diff;
    std::vector<cv::Point> ext;
    void chainAngleDiff(void);
    void firstLoop(void);
    void secLoop(void);
public:
    ExtremesObst2Point(cv::Point2i crit, std::vector<Chain> c);
    std::vector<cv::Point2i> getExt(void);
};

#endif // OBS_EXTREMES_HPP
