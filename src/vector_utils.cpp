#include "vector_utils.hpp"
#include <cmath>

using namespace std;

static const double PI = 3.141592653589793;

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
