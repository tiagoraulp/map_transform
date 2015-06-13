#include "CritPoints.hpp"

CritPoints::CritPoints(cv::Mat map, cv::Mat reach, int rs): r_map(reach), map_or(map), infl(rs)
{
}

cv::Point2i CritPoints::getCrit(void)
{
    return critP;
}
vector<float> CritPoints::getExtremes(void)
{
    return extremes;
}
vector<cv::Point2i> CritPoints::getExtremesP(void)
{
    return extremesP;
}
unsigned int CritPoints::getObt(void)
{
    return obt;
}


cv::Point2i CritPoints::find_crit_point(vector<cv::Point> frontier_p)
{
    FindMin<int> min_y, min_x;
    FindMax<int> max_y, max_x;

    for(unsigned int j=0;j<frontier_p.size();j++){
        max_x.iter(frontier_p[j].x);
        min_x.iter(frontier_p[j].x);
        max_y.iter(frontier_p[j].y);
        min_y.iter(frontier_p[j].y);
    }

    FindMin<double, cv::Point2i> crit;

    for(int x=max(min_x.getVal()-infl,0);x<min(max_x.getVal()+infl,r_map.rows);x++)
    {
        for(int y=max(min_y.getVal()-infl,0);y<min(max_y.getVal()+infl,r_map.cols);y++)
        {
            if(r_map.at<uchar>(x,y)==255)
            {
                double sum=0;
                for(unsigned int l=0;l<frontier_p.size();l++){
                    sum+=(frontier_p[l].x-x)*(frontier_p[l].x-x)+(frontier_p[l].y-y)*(frontier_p[l].y-y);
                }
                crit.iter(sum,cv::Point2i(x,y));
            }
        }
    }

    critP=crit.getP();
    frontier=frontier_p;

    return critP;
}


cv::Point2i CritPoints::find_extreme(cv::Point2i pt, float a0, float a1, bool special)
{
    FindMin<float,cv::Point2i> mp;
    for(int rowx=max((pt.x-1),0);rowx<=min((pt.x+1),map_or.rows-1);rowx++)
    {
        for(int coly=max((pt.y-1),0);coly<=min((pt.y+1),map_or.cols-1);coly++)
        {
            float angle=atan2(coly-critP.y,rowx-critP.x);
            bool inside_region;

            if(special)
                inside_region=(
                                ( (a0>a1) && (angle<a1) && ( (a0-angle)<PI ) ) ||
                                ( (a0<angle) && (angle<a1) && ( (a0+2*PI-angle)<PI ) ) ||
                                ( (a0>a1) && (angle>a0) && ( (a0+2*PI-angle)<PI ) ) ||
                                ( (angle>a1) && (a0<a1) && ( (angle-a0)<PI ) ) ||
                                ( (angle<a0) && (a0<a1) && ( (angle+2*PI-a0)<PI ) ) ||
                                ( (angle>a1) && (a0>angle) && ( (angle+2*PI-a0)<PI ) )
                              );
            else
            {
                if(a0<a1)
                    inside_region=(angle>a0 && angle<a1);
                else
                    inside_region=(angle>a0 || angle<a1);
            }

            if(inside_region  && map_or.at<uchar>(rowx,coly)==0)
            {
                float dist=(rowx-critP.x)*(rowx-critP.x)+(coly-critP.y)*(coly-critP.y);
                mp.iter(dist,cv::Point2i(rowx,coly));
            }
        }
    }

    return mp.getP();


}

float CritPoints::getAngle(cv::Point2i pt){
    return atan2(pt.y-critP.y,pt.x-critP.x);
}

void CritPoints::extremePoints(cv::Point2i pt, cv::Point2i pt2, float a0, float a1, bool first)
{
    extremes.clear();
    extremesP.clear();
    float e;
    cv::Point2i ptf;

    if(first)
        ptf=find_extreme(pt, -4*PI, 4*PI);
    else
        ptf=find_extreme(pt, a0, a1);

    extremesP.push_back(ptf);
    e=getAngle(ptf);
    extremes.push_back(e);

    if(first)
        ptf=find_extreme(pt2, extremes[0], a1, true);
    else
        ptf=find_extreme(pt2, a0, a1);

    e=getAngle(ptf);

    if( e<extremes[0] )
    {
        extremes.insert(extremes.begin(),e);
        extremesP.insert(extremesP.begin(),ptf);
    }
    else
    {
        extremes.push_back(e);
        extremesP.push_back(ptf);
    }

    if( (extremes[1]-extremes[0])>PI )
        obt=0;
    else
        obt=1;
}

vector<float> CritPoints::frontier_extremes(void)
{
    Find_Obtuse_Angle oa;

    for(unsigned int l=0;l<frontier.size();l++)
    {
        float angle=atan2(frontier[l].y-critP.y,frontier[l].x-critP.x);

        oa.iter(angle,cv::Point2i(frontier[l].x,frontier[l].y));
    }

    if(oa.angles.getSize()==1)
    {
        extremePoints(oa.angles.getP(0), oa.angles.getP(0), 0, oa.angles.getVal(0), true);
    }
    else if(oa.angles.getSize()>1)
    {
        if(oa.getObt()==(oa.angles.getSize()-1))
        {
            extremePoints(oa.angles.getP(oa.getObt()), oa.angles.getP(0), oa.angles.getVal(oa.getObt()), oa.angles.getVal(0));
        }
        else
        {
            extremePoints(oa.angles.getP(oa.getObt()), oa.angles.getP(oa.getObt()+1), oa.angles.getVal(oa.getObt()), oa.angles.getVal(oa.getObt()+1));
        }
    }

    return extremes;
}
