#ifndef OBTUSEANGLE_HPP
#define OBTUSEANGLE_HPP

#include "vector_utils.hpp"
#include <opencv2/core/core.hpp>

extern const double PI;

class Find_Obtuse_Angle
{
private:
    unsigned int obt_angle;
public:
    OrderedCresc<float, cv::Point2i> angles;

    Find_Obtuse_Angle(void);
    void initialize(float a0, float a1);
    void updateOA(int i0, int i1, int i2, bool p1, bool p2);
    void iter(float angle, cv::Point2i pt);
    unsigned int getObt(void);
    void clear(void);
};


#endif // OBTUSEANGLE_HPP
