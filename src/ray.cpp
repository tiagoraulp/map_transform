#include "ray.hpp"


bool raytracing(cv::Mat *map, cv::Point2i opt, cv::Point2i ref, cv::Point2i dest, float dist_t, bool (*func)(cv::Mat *,cv::Point2i,cv::Point2i))
{
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

        tempx=tempx+temp*cos_ang;
        tempy=tempy+temp*sin_ang;

        if(cos_ang!=0)
        {
            temp_tx=(p_x+sign_x*0.5-tempx)/cos_ang;
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

bool raytracing(cv::Mat map, int opt_x, int opt_y, int dest_x, int dest_y)
{
    float dist_t=sqrt( (dest_x-opt_x)*(dest_x-opt_x)+(dest_y-opt_y)*(dest_y-opt_y) );
    return raytracing(&map, cv::Point2i(opt_x,opt_y), cv::Point2i(opt_x,opt_y), cv::Point2i(dest_x,dest_y), dist_t, &checkMap);
}

void raytracing(cv::Mat* map, cv::Point2i opt, cv::Point2i ref, cv::Point2i dest, float dist_t)
{
    raytracing(map, opt, ref, dest, dist_t, &print2Map);
}

cv::Mat brute_force(cv::Mat map, cv::Mat reach, int defl)
{
    cv::Mat result=map.clone();

    for(int i=0;i<map.rows;i++)
    {
        for(int j=0;j<map.cols;j++)
        {
            if(map.at<uchar>(i,j)==0)
                continue;

            bool stop=false;

            for(int ii=0;ii<reach.rows;ii++)
            {
                for(int jj=0;jj<reach.cols;jj++)
                {
                    if(reach.at<uchar>(ii,jj)==0)
                        continue;
                    if( ( (i-ii)*(i-ii)+(j-jj)*(j-jj) )>defl*defl )
                        continue;
                    if( raytracing(map,ii,jj,i,j) )
                    {
                        stop=true;
                        break;
                    }
                }
                if(stop)
                    break;
            }
            if(stop)
                continue;

            result.at<uchar>(i,j)=0;
        }
    }

    return result;
}


bool bfo_iter(cv::Mat map, cv::Mat reach, int defl, int i, int j, int ii, int jj)
{
    if( (ii)<map.rows &&  (ii)>=0 && (jj)<map.rows &&  (jj)>=0 )
    {
        if(reach.at<uchar>(ii,jj)==0)
            return false;
        else
        {
            if( ( (i-ii)*(i-ii)+(j-jj)*(j-jj) )>defl*defl )
                return false;
            else
            {
                if( raytracing(map,ii,jj,i,j) )
                {
                    return true;
                }
            }
        }
    }
    return false;
}

bool bfo(cv::Mat map, cv::Mat reach, int defl, int i, int j)
{
    for(int r=0; r<=defl; r++)
    {
        if(r==0)
        {
            if(reach.at<uchar>(i,j)==0)
                continue;
            else
            {
                return true;
            }
        }
        else
        {
            for(int p=-r;p<r;p++)
            {
                int ii=i-r, jj=j+p;
                if(bfo_iter(map, reach, defl, i, j, ii, jj))
                    return true;

                ii=i+p, jj=j+r;
                if(bfo_iter(map, reach, defl, i, j, ii, jj))
                    return true;

                ii=i+r, jj=j-p;
                if(bfo_iter(map, reach, defl, i, j, ii, jj))
                    return true;

                ii=i-p, jj=j-r;
                if(bfo_iter(map, reach, defl, i, j, ii, jj))
                    return true;
            }
        }
    }

    return false;
}


cv::Mat brute_force_opt(cv::Mat map, cv::Mat reach, int defl)
{
    cv::Mat result=map.clone();

    for(int i=0;i<map.rows;i++)
    {
        for(int j=0;j<map.cols;j++)
        {
            if(map.at<uchar>(i,j)==0)
                continue;

            if(bfo( map, reach, defl, i, j) )
                continue;

            result.at<uchar>(i,j)=0;
        }
    }

    return result;
}

cv::Mat brute_force_opt_act(cv::Mat map, cv::Mat reach, cv::Mat act, int defl)
{
    cv::Mat result=map.clone();

    for(int i=0;i<map.rows;i++)
    {
        for(int j=0;j<map.cols;j++)
        {
            if(map.at<uchar>(i,j)==0)
                continue;
            if(act.at<uchar>(i,j)==255)
                continue;

            if(bfo( map, reach, defl, i, j) )
                continue;

            result.at<uchar>(i,j)=0;
        }
    }

    return result;
}
