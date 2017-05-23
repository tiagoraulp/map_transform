#include "ray.hpp"
#include <cmath>

using namespace std;

/// opt and dest define angle
/// ref is starting point
/// dist_t is maximum distance from opt

bool raytracing(cv::Mat *map, cv::Point2i opt, cv::Point2i ref, cv::Point2i dest, float dist_t, bool (*func)(cv::Mat *,cv::Point2i,cv::Point2i),  cv::Mat & test_pt, vector<cv::Point> & list)
{
    bool boost=true, boost_list=true;
    if(test_pt.rows!=map->rows && test_pt.cols!=map->cols)
        boost=false;
    if(list.size()==0)
        boost_list=false;

    float angle=atan2(dest.y-opt.y, dest.x-opt.x);

    float cos_ang=cos(angle);
    float sin_ang=sin(angle);

    float sign_x=0; if(cos_ang!=0) sign_x=cos_ang/abs(cos_ang);
    float sign_y=0; if(sin_ang!=0) sign_y=sin_ang/abs(sin_ang);

    int p_x=ref.x;
    int p_y=ref.y;

    float temp, tempx=ref.x, tempy=ref.y, temp_tx, temp_ty;

    if(cos_ang!=0)
    {
        temp_tx=sign_x*0.5/cos_ang;
        if(sin_ang!=0)
        {
            temp_ty=sign_y*0.5/sin_ang;
            if(temp_tx<temp_ty)
                temp=temp_tx;
            else
                temp=temp_ty;
        }
        else
            temp=temp_tx;
    }
    else
        temp=sign_y*0.5/sin_ang;

    int prev_px=p_x;
    int prev_py=p_y;

    if(sign_x==0)
        p_x=p_x;
    else
        p_x=sign_x*((int)round(sign_x*(tempx+temp*cos_ang)+p_x)-p_x);

    if(sign_y==0)
        p_y=p_y;
    else
        p_y=sign_y*((int)round(sign_y*(tempy+temp*sin_ang)+p_y)-p_y);

    float dist=sqrt( (ref.x-opt.x)*(ref.x-opt.x)+(ref.y-opt.y)*(ref.y-opt.y) );

    while( (dist+temp)<=dist_t && p_x>=0 && p_x<map->rows && p_y>=0 && p_y<map->cols )
    {
        dist=dist+temp;

        if(!(*func)(map, cv::Point2i(p_x,p_y), cv::Point2i(prev_px,prev_py)))
            return false;

        if(boost)
            test_pt.at<uchar>(p_x,p_y)=0;

        if(boost_list)
            list.push_back(cv::Point(p_x,p_y));

        tempx=tempx+temp*cos_ang;
        tempy=tempy+temp*sin_ang;

        if(cos_ang!=0)
        {
            temp_tx=(p_x+sign_x*0.5-tempx)/cos_ang;
            /// TODO: understand this. only equal to zero if rounding before has error.
            /// And in that case, correcting 0 might not be good, and maybe it would be self correcting if temp=0.
            /// But if that was the case, why did I add this here?... Probably it was stuck in infinite cycle!
            /// if solution is not correct, there would be lines jumping middle points (both when printing or checking)...
            if(temp_tx==0)
                temp_tx=sign_x*1/cos_ang;
            if(sin_ang!=0)
            {
                temp_ty=(p_y+sign_y*0.5-tempy)/sin_ang;
                if(temp_ty==0)
                    temp_ty=sign_y*1/sin_ang;

                if(temp_tx<temp_ty)
                    temp=temp_tx;
                else
                    temp=temp_ty;
            }
            else
                temp=temp_tx;
        }
        else
        {
            temp=(p_y+sign_y*0.5-tempy)/sin_ang;
            if(temp==0)
                temp=sign_y*1/sin_ang;
        }

        prev_px=p_x;
        prev_py=p_y;

        if(sign_x==0)
            p_x=p_x;
        else
            p_x=sign_x*((int)round(sign_x*(tempx+temp*cos_ang)+p_x)-p_x);

        if(sign_y==0)
            p_y=p_y;
        else
            p_y=sign_y*((int)round(sign_y*(tempy+temp*sin_ang)+p_y)-p_y);
    }
    return true;
}


bool checkMap(cv::Mat *map, cv::Point2i p, cv::Point2i prev)
{
    if( map->at<uchar>(p.x,p.y)==0 )
        return false;

    if( (abs(prev.x-p.x)+abs(prev.y-p.y))==2 )
    {
        if( map->at<uchar>(prev.x,p.y)==0 )
            return false;

        if( map->at<uchar>(p.x,prev.y)==0 )
            return false;
    }

    return true;
}


bool print2Map(cv::Mat *map, cv::Point2i p, cv::Point2i prev)//, cv::Point2i prev)
{
    map->at<uchar>(p.x,p.y)=0;
    return true;
}

bool raytracing(cv::Mat map, int opt_x, int opt_y, int dest_x, int dest_y, cv::Mat & test_pt, vector<cv::Point> & list)
{
    float dist_t=sqrt( (dest_x-opt_x)*(dest_x-opt_x)+(dest_y-opt_y)*(dest_y-opt_y) );
    return raytracing(&map, cv::Point2i(opt_x,opt_y), cv::Point2i(opt_x,opt_y), cv::Point2i(dest_x,dest_y), dist_t, &checkMap, test_pt, list);
}

bool raytracing(cv::Mat map, int opt_x, int opt_y, int dest_x, int dest_y, bool t, cv::Mat & test_pt, vector<cv::Point> & list)
{
    float dist_t=sqrt( (dest_x-opt_x)*(dest_x-opt_x)+(dest_y-opt_y)*(dest_y-opt_y) );
    if(opt_x<map.rows && opt_x>=0 && opt_y<map.cols && opt_y>=0 && dest_x<map.rows && dest_x>=0 && dest_y<map.cols && dest_y>=0)
        return raytracing(&map, cv::Point2i(opt_x,opt_y), cv::Point2i(opt_x,opt_y), cv::Point2i(dest_x,dest_y), dist_t, &checkMap, test_pt, list);
    else
        return false;
}

bool raytracing(cv::Mat map, int opt_x, int opt_y, int dest_x, int dest_y, bool t)
{
    cv::Mat temp=cv::Mat(0,0,CV_8UC1);
    vector<cv::Point> list;
    return raytracing(map, opt_x, opt_y, dest_x, dest_y, temp, list);
}

void raytracing(cv::Mat* map, cv::Point2i opt, cv::Point2i ref, cv::Point2i dest, float dist_t)
{
    cv::Mat temp=cv::Mat(0,0,CV_8UC1);
    vector<cv::Point> list;
    raytracing(map, opt, ref, dest, dist_t, &print2Map, temp, list);
}

