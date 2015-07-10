#include "obs_extremes.hpp"


ExtremesObst2Point::ExtremesObst2Point(cv::Point2i crit, vector<Chain> c): critP(crit), ch(c)
{
    chainAngleDiff();
    ext.clear();
    prev_diff=0;
    firstLoop();
    secLoop();
}

vector<cv::Point2i> ExtremesObst2Point::getExt(void)
{
    return ext;
}

void ExtremesObst2Point::chainAngleDiff(void)
{
    diffAngle.clear();
    float prev_angle, act_angle;
    for(unsigned int c=0;c<ch.size();c++)
    {
        act_angle=atan2(ch[c].y-critP.y,ch[c].x-critP.x);
        if(c>0)
        {
            float diff_angle=act_angle-prev_angle;

            if(diff_angle<-PI)
                diff_angle+=2*PI;
            else if (diff_angle>PI)
                diff_angle-=2*PI;

            diffAngle.push_back(diff_angle);
        }
        prev_angle=act_angle;
    }
}

void ExtremesObst2Point::firstLoop(void)
{
    for(unsigned int c=0;c<diffAngle.size();c++)
    {
        if(c>0)
        {
            if( diffAngle[c]*prev_diff<0)
            {
                ext.push_back(cv::Point(ch[c].x,ch[c].y));
                prev_diff=diffAngle[c];
            }
            if(prev_diff==0)
                prev_diff=diffAngle[c];
        }
        else
            prev_diff=diffAngle[c];
    }
}

void ExtremesObst2Point::secLoop(void)
{
    for(unsigned int c=0;c<diffAngle.size();c++)
    {
        if( diffAngle[c]*prev_diff<0)
        {
            ext.push_back(cv::Point(ch[c].x,ch[c].y));
            break;
        }
        else if(diffAngle[c]*prev_diff>0)
        {
            break;
        }
    }
}

