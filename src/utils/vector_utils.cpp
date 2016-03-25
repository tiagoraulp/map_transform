#include "vector_utils.hpp"
#include <cmath>

using namespace std;

static const double PI = 3.141592653589793;

vector<cv::Point> bf_hlx(int defl)
{
    vector<cv::Point> hlx;
    for(int r=0; r<=defl; r++)
    {
        if(r==0)
        {
            hlx.push_back(cv::Point(0,0));
        }
        else
        {
            for(int p=-r;p<r;p++)
            {
                int ii=0-r, jj=0+p;
                hlx.push_back(cv::Point(ii,jj));

                ii=0+p, jj=0+r;
                hlx.push_back(cv::Point(ii,jj));

                ii=0+r, jj=0-p;
                hlx.push_back(cv::Point(ii,jj));

                ii=0-p, jj=0-r;
                hlx.push_back(cv::Point(ii,jj));
            }
        }
    }
    return hlx;
}

int boundPos(int x, int max)
{
    if(x>=max)
        x=max-1;
    if(x<0)
        x=0;
    return x;
}

int incAngle(int p, int num)
{
    int u;

    if(p==(num-1))
        u=0;
    else
        u=p+1;

    return u;
}

int decAngle(int p, int num)
{
    int l;

    if(p==0)
        l=num-1;
    else
        l=p-1;

    return l;
}

double boundAngle(double x, double v)
{
    if(x<0)
    {
        while(x<0)
            x+=v;
    }
    else
    {
        while(x>v)
            x-=v;
    }

    return x;
}

int boundValue(int x, int v)
{
    if(x<0)
    {
        while(x<0)
            x+=v;
    }
    else
    {
        while(x>v)
            x-=v;
    }

    return x;
}

double boundAngleN(double x, double v)
{
    if(x<-v)
    {
        while(x<-v)
            x+=2*v;
    }
    else
    {
        while(x>v)
            x-=2*v;
    }

    return x;
}

int boundValueN(int x, int v)
{
    if(x<-v)
    {
        while(x<-v)
            x+=2*v;
    }
    else
    {
        while(x>v)
            x-=2*v;
    }

    return x;
}

double boundAngleR(double x)
{
    return boundAngle(x, 2*PI);
}

double boundAngleD(double x)
{
    return boundAngle(x, 360);
}

double boundAngleRN(double x)
{
    return boundAngleN(x, PI);
}

double boundAngleDN(double x)
{
    return boundAngleN(x, 180);
}

double angleDiff(double a, double b, int res)
{
    return abs(boundAngleN( (a-b),((double)res)/2.0));
}

int angleDiff(int a, int b, int res)
{
    return (int)round(angleDiff((double) a,(double) b, res));
}

int angleD2I(double rtrd, int angle_res)
{
    rtrd=boundAngleD(rtrd);

    double min_err=0;
    int m_a=0;

    for(int i=0;i<angle_res;i++)
    {
        if(i==0)
        {
            m_a=i;
            min_err=abs(360.0/angle_res*i-rtrd);
        }
        else
        {
            if( abs(360.0/angle_res*i-rtrd)<min_err  )
            {
                m_a=i;
                min_err=abs(360.0/angle_res*i-rtrd);
            }
        }
    }
    if( abs(360.0/angle_res*angle_res-rtrd)<min_err  )
    {
        m_a=0;
    }

    return m_a;
}

int angleR2I(double rtrd, int angle_res)
{
    return angleD2I(angleR2D(rtrd), angle_res);
}

double pi(void)
{
    return PI;
}

cv::Point2f operator*(cv::Mat M, const cv::Point2f p){
    cv::Mat src(3/*rows*/,1 /* cols */,CV_64F);

    src.at<double>(0,0)=p.x;
    src.at<double>(1,0)=p.y;
    src.at<double>(2,0)=1.0;

    cv::Mat dst = M*src; //USE MATRIX ALGEBRA
    return cv::Point2f(dst.at<double>(0,0),dst.at<double>(1,0));
}
