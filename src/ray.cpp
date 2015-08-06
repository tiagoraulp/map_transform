#include "ray.hpp"

#include <cmath>

using namespace std;

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

bool raytracing(cv::Mat map, int opt_x, int opt_y, int dest_x, int dest_y, bool t)
{
    float dist_t=sqrt( (dest_x-opt_x)*(dest_x-opt_x)+(dest_y-opt_y)*(dest_y-opt_y) );
    if(opt_x<map.rows && opt_x>=0 && opt_y<map.cols && opt_y>=0 && dest_x<map.rows && dest_x>=0 && dest_y<map.cols && dest_y>=0)
        return raytracing(&map, cv::Point2i(opt_x,opt_y), cv::Point2i(opt_x,opt_y), cv::Point2i(dest_x,dest_y), dist_t, &checkMap);
    else
        return false;
}

void raytracing(cv::Mat* map, cv::Point2i opt, cv::Point2i ref, cv::Point2i dest, float dist_t)
{
    raytracing(map, opt, ref, dest, dist_t, &print2Map);
}

bool bf_sq(cv::Mat map, cv::Mat reach, int defl, int i, int j)
{
    bool stop=false;

    for(int ii=0;ii<reach.rows;ii++)
    {
        for(int jj=0;jj<reach.cols;jj++)
        {
            if( ( (i-ii)*(i-ii)+(j-jj)*(j-jj) )>defl*defl )
                continue;
            if(reach.at<uchar>(ii,jj)==0)
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

    return stop;
}

bool bf_sq(cv::Mat map, vector<cv::Mat> reach, Elem sensor, int i, int j)
{
    bool stop=false;

    for(unsigned int a=0; a<reach.size();a++)
    {
        for(int ii=0;ii<reach[a].rows;ii++)
        {
            for(int jj=0;jj<reach[a].cols;jj++)
            {
                if(reach[a].at<uchar>(ii,jj)==0)
                    continue;

                if( (i+sensor.pu-ii+sensor.pt.x)<sensor.elems[a].rows &&  (i+sensor.pu-ii+sensor.pt.x)>=0 && (j+sensor.pl-jj+sensor.pt.y)<sensor.elems[a].cols &&  (j+sensor.pl-jj+sensor.pt.y)>=0 )
                {
                    if(sensor.elems[a].at<uchar>(i+sensor.pu-ii+sensor.pt.x, j+sensor.pl-jj+sensor.pt.y)==0)
                    {
                        continue;
                    }
                    else
                    {
                        if( raytracing(map,ii-sensor.pu+sensor.pt2[a].x-sensor.pt.x,jj-sensor.pl+sensor.pt2[a].y-sensor.pt.y,i,j, true) )
                        {
                            stop=true;
                            break;
                        }
                    }
                }
                else
                {
                    continue;
                }
            }
            if(stop)
                break;
        }
        if(stop)
            break;
    }

    return stop;
}

bool bfo_iter(cv::Mat map, cv::Mat reach, Elem sensor, int i, int j, int ii, int jj, unsigned int a)
{
    if( (ii+sensor.pu)<reach.rows &&  (ii+sensor.pu)>=0 && (jj+sensor.pl)<reach.cols &&  (jj+sensor.pl)>=0 )
    {

        if(reach.at<uchar>(ii+sensor.pu,jj+sensor.pl)==0)
        {
            return false;
        }
        else
        {
            if( (i-ii+sensor.pt.x)<sensor.elems[a].rows &&  (i-ii+sensor.pt.x)>=0 && (j-jj+sensor.pt.y)<sensor.elems[a].cols &&  (j-jj+sensor.pt.y)>=0 )
            {
                if(sensor.elems[a].at<uchar>(i-ii+sensor.pt.x, j-jj+sensor.pt.y)==0)
                {
                    return false;
                }
                else
                {
                    if( raytracing(map,ii+sensor.pt2[a].x-sensor.pt.x,jj+sensor.pt2[a].y-sensor.pt.y,i,j, true) )
                    {
                        return true;
                    }
                }
            }
            else
            {
                return false;
            }
        }
    }
    return false;
}

bool bfo_iter(cv::Mat map, cv::Mat reach, int defl, int i, int j, int ii, int jj)
{
    if( (ii)<reach.rows &&  (ii)>=0 && (jj)<reach.cols &&  (jj)>=0 )
    {
        if(reach.at<uchar>(ii,jj)==0)
        {
            return false;
        }
        else
        {
            if( ( (i-ii)*(i-ii)+(j-jj)*(j-jj) )>defl*defl )
            {
                return false;
            }
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
            {
                continue;
            }
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



bool bfo(cv::Mat map, vector<cv::Mat> reach, Elem sensor, int i, int j)
{
    int defl=sensor.pb;
    for(int r=0; r<=defl; r++)
    {
        for(unsigned int a=0; a<reach.size(); a++)
        {
            if(r==0)
            {
                if(bfo_iter(map, reach[a], sensor, i, j, i, j, a))
                    return true;
                else
                {
                    continue;
                }
            }
            else
            {
                for(int p=-r;p<r;p++)
                {
                    int ii=i-r, jj=j+p;
                    if(bfo_iter(map, reach[a], sensor, i, j, ii, jj,a))
                        return true;

                    ii=i+p, jj=j+r;
                    if(bfo_iter(map, reach[a], sensor, i, j, ii, jj,a))
                        return true;

                    ii=i+r, jj=j-p;
                    if(bfo_iter(map, reach[a], sensor, i, j, ii, jj,a))
                        return true;

                    ii=i-p, jj=j-r;
                    if(bfo_iter(map, reach[a], sensor, i, j, ii, jj,a))
                        return true;
                }
            }
        }
    }

    return false;
}

bool bfo(cv::Mat map, vector<cv::Point> reach, int defl, int i, int j)
{
    for(unsigned int r=0; r<reach.size(); r++)
    {
        int ii=reach[r].x;
        int jj=reach[r].y;

        if( ( (i-ii)*(i-ii)+(j-jj)*(j-jj) )>defl*defl )
            continue;
        else
        {
            if( raytracing(map,ii,jj,i,j) )
            {
                return true;
            }
        }
    }

    return false;
}

bool bfo(cv::Mat map, vector<cv::Point3i> reach, Elem sensor, int i, int j)
{
    for(unsigned int r=0; r<reach.size(); r++)
    {
        int ii=reach[r].x;
        int jj=reach[r].y;
        int a=reach[r].z;

        if( (i+sensor.pu-ii+sensor.pt.x)<sensor.elems[a].rows &&  (i+sensor.pu-ii+sensor.pt.x)>=0 && (j+sensor.pl-jj+sensor.pt.y)<sensor.elems[a].cols &&  (j+sensor.pl-jj+sensor.pt.y)>=0 )
        {
            if(sensor.elems[a].at<uchar>(i+sensor.pu-ii+sensor.pt.x, j+sensor.pl-jj+sensor.pt.y)==0)
            {
                continue;
            }
            else
            {
                if( raytracing(map,ii-sensor.pu+sensor.pt2[a].x-sensor.pt.x,jj-sensor.pl+sensor.pt2[a].y-sensor.pt.y,i,j, true) )
                {
                    return true;
                }
            }
        }
        else
        {
            continue;
        }

    }
    return false;
}

bool bf_sq(cv::Mat map, vector<cv::Point> reach, int defl, int i, int j)
{
    return bfo(map, reach, defl, i, j);
}

bool bf_sq(cv::Mat map, vector<cv::Point3i> reach, Elem defl, int i, int j)
{
    return bfo(map, reach, defl, i, j);
}

template <typename T, typename T2>
cv::Mat brute_force(cv::Mat map, T reach, T2 defl, bool opt, cv::Mat act)
{
    cv::Mat result=map.clone();

    bool act_test;

    if(act.rows!=map.rows || act.cols!=map.cols)
        act_test=false;
    else
        act_test=true;


    if(opt)
    {

    }

    for(int i=0;i<map.rows;i++)
    {
        for(int j=0;j<map.cols;j++)
        {
            //cout<<i<<" "<<j<<endl;
            if(map.at<uchar>(i,j)==0)
                continue;

            if(act_test)
            {
                if(act.at<uchar>(i,j)==255)
                    continue;
            }

            if(opt)
            {
                if(bfo( map, reach, defl, i, j ) )
                    continue;
            }
            else
            {
                if(bf_sq( map, reach, defl, i, j) )
                    continue;
            }


            result.at<uchar>(i,j)=0;
        }
    }

    return result;

}

template cv::Mat brute_force<cv::Mat, int>(cv::Mat map, cv::Mat reach, int defl, bool opt, cv::Mat act);
template cv::Mat brute_force<vector<cv::Mat>, Elem>(cv::Mat map, vector<cv::Mat> reach, Elem defl, bool opt, cv::Mat act);
template cv::Mat brute_force<vector<cv::Point>, int>(cv::Mat map, vector<cv::Point> reach, int defl, bool opt, cv::Mat act);
template cv::Mat brute_force<vector<cv::Point3i>, Elem>(cv::Mat map, vector<cv::Point3i> reach, Elem defl, bool opt, cv::Mat act);
