#include "CritPoints.hpp"
#include "ray.hpp"
#include "obtuseAngle.hpp"

#include <cmath>
#include <iostream>

using namespace std;

static const double PI = 3.141592653589793;

CritPointsAS::CritPointsAS(cv::Mat map, vector<cv::Mat> reach, Elem sensor_ev, std::vector<int> sens_area): CritPoints(map, reach[0], sensor.pb), reach3(reach), sensor(sensor_ev), sens_hist(sens_area)
{
    critP3=cv::Point3i(-1,-1,-1);
}

cv::Point3i CritPointsAS::find_crit_point(ClusterLists cluster_p)
{
    if(cluster_p.extremes.size()!=2)
    {
        critP3=cv::Point3i(-1,-1,-1);
        critP=cv::Point2i(-1,-1);

        frontier=cluster_p.cluster;

        return critP3;
    }

    float dux=cluster_p.extremes[0].x-cluster_p.extremes[1].x;
    float duy=cluster_p.extremes[0].y-cluster_p.extremes[1].y;

    FindMin<int> min_y, min_x;
    FindMax<int> max_y, max_x;
    double mean_x=0, mean_y=0;

    vector<cv::Point> frontier_p=cluster_p.cluster;

    for(unsigned int j=0;j<frontier_p.size();j++){
        max_x.iter(frontier_p[j].x);
        min_x.iter(frontier_p[j].x);
        max_y.iter(frontier_p[j].y);
        min_y.iter(frontier_p[j].y);
        mean_x+=frontier_p[j].x;
        mean_y+=frontier_p[j].y;
    }
    cv::Point center((int)round(mean_x/frontier_p.size()),(int)round(mean_y/frontier_p.size()));

    float d0=2*center.x-cluster_p.extremes[0].x-cluster_p.extremes[1].x;
    float d1=2*center.y-cluster_p.extremes[0].y-cluster_p.extremes[1].y;

    float nx=duy;
    float ny=-dux;

    if((nx*d0/2+ny*d1/2)<-0.05)
    {
        nx=-nx;
        ny=-ny;
    }
    else if( (nx*d0+ny*d1)<0.05 )
    {
        nx=0;
        ny=0;
    }

//    FindMin<float, cv::Point> cl_center;

//    for(unsigned int j=0;j<frontier_p.size();j++){
//        cl_center.iter( (frontier_p[j].x-center.x)*(frontier_p[j].x-center.x)+(frontier_p[j].y-center.y)*(frontier_p[j].y-center.y), frontier_p[j]);
//    }

//    cv::Point centerF=cl_center.getP();

    int m_x=max(max_x.getVal()-center.x,center.x-min_x.getVal());
    int m_y=max(max_y.getVal()-center.y,center.y-min_y.getVal());

    int md=(int)sqrt(m_x*m_x+m_y*m_y);

    int ext=2;

    vector<cv::Point> sq=bf_hlx(md+ext+sensor.pb);

    FindMax<double, cv::Point3i> crit;

    for(unsigned int r=0; r<sq.size(); r++)
    {
        int i=center.x+sq[r].x, j=center.y+sq[r].y;

        //int refa=angleD2I(angleR2D(atan2(center.y-j, center.x-i)),reach3.size());

        for(unsigned int a=0; a<reach3.size(); a++)
        {
            if( (i+sensor.pu)<reach3[a].rows &&  (i+sensor.pu)>=0 && (j+sensor.pl)<reach3[a].cols &&  (j+sensor.pl)>=0 )
            {
                if(reach3[a].at<uchar>(i+sensor.pu,j+sensor.pl)==255)
                {
                    //int angdiff=angleDiff(refa, a, reach3.size());

                    //if(angdiff<=( (int)round(((double)reach3.size())*0.20 )))
                    //{
                        //double est_area=((center.x-i)*(center.x-i)+(center.x-i)*(center.y-j))*PI;

                        int ii=i+sensor.pt2[a].x-sensor.pt.x;
                        int jj=j+sensor.pt2[a].y-sensor.pt.y;

                        vector<double> angle;
                        angle.assign(2,0);
                        double angleA, angleB;//, angleC;

                        angleA=atan2(cluster_p.extremes[0].y-jj, cluster_p.extremes[0].x-ii);

                        angleB=atan2(cluster_p.extremes[1].y-jj, cluster_p.extremes[1].x-ii);

                        //angleC=atan2(center.y-jj, center.x-ii);
//                        angleC=atan2(centerF.y-jj, centerF.x-ii);

//                        int start;

                        if(angleA<angleB)
                        {
                            angle[0]=angleA;
                            angle[1]=angleB;
                        }
                        else
                        {
                            angle[0]=angleB;
                            angle[1]=angleA;
                        }

//                        if(angleC>=angle[0] && angleC<=angle[1])
//                            start=0;
//                        else
//                            start=1;

                        double angle_diff;

//                        if(start==0)
//                        {
// //                            //if( (angle[1]-angle[0]) > PI )
// //                            if( ((angle[1]-angleC) > PI) || ((angleC-angle[0]) > PI) )
// //                                continue;
// //                            else
//                                angle_diff=angle[1]-angle[0];
//                        }
//                        else
//                        {
//                            //if( (2*PI-angle[1]+angle[0]) > PI )
//                            //    continue;
//                            //else
//                            //    angle_diff=2*PI-angle[1]+angle[0];

// //                            if(angleC>angle[1])
// //                            {
// //                                if( ((angleC-angle[1]) > PI) || ((2*PI-angleC+angle[0]) > PI) )
// //                                    continue;
// //                                else
// //                                    angle_diff=2*PI-angle[1]+angle[0];
// //                            }
// //                            else
// //                            {
// //                                if( ((angle[0]-angleC) > PI) || ((2*PI-angle[1]+angleC) > PI) )
// //                                    continue;
// //                                else
//                                    angle_diff=2*PI-angle[1]+angle[0];
// //                            }
//                        }

                        //float d0;//=(cluster_p.extremes[0].y-j)*(cluster_p.extremes[0].y-j)+(cluster_p.extremes[0].x-i)*(cluster_p.extremes[0].x-i);
                        //float d1;//=(cluster_p.extremes[1].y-j)*(cluster_p.extremes[1].y-j)+(cluster_p.extremes[1].x-i)*(cluster_p.extremes[1].x-i);
                        //float d;//=(center.y-j)*(center.y-j)+(center.x-i)*(center.x-i);

                        //if( (d<d1) || (d<d0) )
                        //    continue;

                        //d0=2*center.x-cluster_p.extremes[0].x-cluster_p.extremes[1].x;
                        //d1=2*center.y-cluster_p.extremes[0].y-cluster_p.extremes[1].y;

                        float d2=cluster_p.extremes[0].y-jj;//0;//=center.y-jj;
                        float d=cluster_p.extremes[0].x-ii;//0;//center.x-ii;

//                        for(unsigned int l=0;l<frontier_p.size();l++){
//                            //sum+=(frontier_p[l].x-ii)*(frontier_p[l].x-ii)+(frontier_p[l].y-jj)*(frontier_p[l].y-jj);
//                            d+=frontier_p[l].x-ii;
//                            d2+=frontier_p[l].y-jj;
//                        }

                        if( ((d*nx+ny*d2)<0) && (nx!=0 || ny!=0) )
                            //continue;
                        {
                            if( (ii<min_x.getVal()) || (ii>max_x.getVal()) || (jj<min_y.getVal()) || (jj>max_y.getVal()) )
                                continue;
                            else
                            {
                                if( (angle[1]-angle[0])>PI )
                                    angle_diff=angle[1]-angle[0];
                                else
                                    angle_diff=2*PI-angle[1]+angle[0];
                            }
                        }
                        else
                        {
                            if( (angle[1]-angle[0])<=PI )
                                angle_diff=angle[1]-angle[0];
                            else
                                angle_diff=2*PI-angle[1]+angle[0];
                        }

                        //if( abs(boundAngleRN(((float)(a))*2*PI/((float)(reach3.size()))-angleC))>(PI/2) )
                        //    continue;

                        //if( (i==178 && j==165 && a==16) || (i==122 && j==23 && a==8) )
                        //    cout<<angle_diff<<endl;


                        //double sum=0;

                        //for(unsigned int l=0;l<frontier_p.size();l++){
                        //    sum+=(frontier_p[l].x-ii)*(frontier_p[l].x-ii)+(frontier_p[l].y-jj)*(frontier_p[l].y-jj);
                        //}

                        crit.iter(angle_diff,cv::Point3i(i+sensor.pu,j+sensor.pl,a));
                    //}
                }
            }
        }
    }

    if(crit.valid())
    {
        critP3=crit.getP();
        critP=cv::Point2i(critP3.x-sensor.pu+sensor.pt2[critP3.z].x-sensor.pt.x, critP3.y-sensor.pl+sensor.pt2[critP3.z].y-sensor.pt.y);
    }
    else
    {
        critP3=cv::Point3i(-1,-1,-1);
        critP=cv::Point2i(-1,-1);
    }

    frontier=frontier_p;

    return critP3;
}

bool CritPointsAS::valid(void)
{
    return !(critP3.x<0 || critP3.y<0 || critP3.z<0);
}

cv::Point3i CritPointsAS::getCrit3(void)
{
    return critP3;
}

CritPoints::CritPoints(cv::Mat map, cv::Mat reach, int rs): r_map(reach), map_or(map), infl(rs)
{
    critP=cv::Point2i(-1,-1);
}

bool CritPoints::valid(void)
{
    return !(critP.x<0 || critP.y<0);
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

    int ext=2;

    for(int x=max(min_x.getVal()-infl-ext,0);x<min(max_x.getVal()+infl+ext,r_map.rows);x++)
    {
        for(int y=max(min_y.getVal()-infl-ext,0);y<min(max_y.getVal()+infl+ext,r_map.cols);y++)
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

    if(crit.valid())
        critP=crit.getP();
    else
        critP=cv::Point2i(-1,-1);

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

            if(inside_region)//  && map_or.at<uchar>(rowx,coly)==0)
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
        obt=1;
        extremes.insert(extremes.begin(),e);
        extremesP.insert(extremesP.begin(),ptf);
    }
    else
    {
        obt=0;
        extremes.push_back(e);
        extremesP.push_back(ptf);
    }

    //if( (extremes[1]-extremes[0])>PI )
    //    obt=0;
    //else
    //    obt=1;
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
