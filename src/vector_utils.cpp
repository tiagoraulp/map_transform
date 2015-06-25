#include "vector_utils.hpp"

int boundPos(int x, int max)
{
    if(x>=max)
        x=max-1;
    if(x<0)
        x=0;
    return x;
}

cv::Mat scaling(cv::Mat or_e, double scale)
{
    int pl, pr, pu, pb;

    cv::Mat res=or_e.clone();

    pu=0;
    pb=or_e.rows*max(0.0,scale-1);
    pl=0;
    pr=or_e.cols*max(0.0,scale-1);

    cv::copyMakeBorder(res,res,pu,pb,pl,pr,cv::BORDER_CONSTANT,cv::Scalar(0));

    res = cv::getRotationMatrix2D(cv::Point(0,0), 0, scale);

    if(scale<1)
    {
        res=res(cv::Rect(0,0,or_e.rows*scale,or_e.rows*scale));
    }
}


