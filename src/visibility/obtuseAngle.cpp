#include "obtuseAngle.hpp"

static const double PI = 3.141592653589793;


Find_Obtuse_Angle::Find_Obtuse_Angle(void)
{
    obt_angle=0;
    angles=OrderedCresc<float,cv::Point2i>();
}

void Find_Obtuse_Angle::initialize(float a0, float a1)
{
    if((a1-a0)>PI)
        obt_angle=0;
    else
        obt_angle=1;
}

void Find_Obtuse_Angle::updateOA(int i0, int i1, int i2, bool p1, bool p2)
{
    float a1,a2;
    if(p1)
        a1=2*PI;
    else
        a1=0;
    if(p2)
        a2=2*PI;
    else
        a2=0;
    if( (angles.getVal(i1)-angles.getVal(i0)+a1)>PI  )
    {
        obt_angle=i0;
    }
    else if( (angles.getVal(i2)-angles.getVal(i1)+a2)>PI  )
    {
        obt_angle=i1;
    }
    else
    {
        FindMax<float> anglediff;
        for(unsigned int aa=0;aa<(angles.getSize()-1);aa++)
        {
            anglediff.iter(angles.getVal(aa+1)-angles.getVal(aa));
        }
        anglediff.iter( (angles.getVal(0)+PI)+(PI-angles.getVal(angles.getSize()-1)) );

        obt_angle=anglediff.getInd();
    }
}

void Find_Obtuse_Angle::iter(float angle, cv::Point2i pt)
{
    unsigned int a=angles.iter(angle,pt);

    if(angles.getSize()==2)
    {
        initialize(angles.getVal(0),angles.getVal(1));
    }
    else if(angles.getSize()>2)
    {
        if(a==(angles.getSize()-1))
        {
            if( (a)==(obt_angle+1) )
            {
                updateOA(obt_angle, a, 0, false, true);
            }
        }
        else
        {
            if(a==(obt_angle+1) )
            {
                updateOA(obt_angle, a, a+1, false, false);
            }
            else if( a==0 && (obt_angle+1)==(angles.getSize()-1) )
            {
                updateOA(obt_angle+1, a, a+1, true, false);
            }
            else if(a<=obt_angle)
            {
                obt_angle+=1;
            }
        }
    }
}

unsigned int Find_Obtuse_Angle::getObt(void)
{
    return obt_angle;
}

void Find_Obtuse_Angle::clear(void)
{
    obt_angle=0;
    angles=OrderedCresc<float,cv::Point2i>();
}
