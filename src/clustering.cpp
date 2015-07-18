#include "clustering.hpp"
#include "vector_utils.hpp"

#include <iostream>

using namespace std;

template <typename T>
Cluster<T>::Cluster()
{
    frontier.clear();
    rest.clear();
}

template <typename T>
void Cluster<T>::append(Cluster<T> b)
{
    frontier.insert(frontier.end(),b.frontier.begin(),b.frontier.end());
    rest.insert(rest.end(),b.rest.begin(),b.rest.end());
}

template <>
void Cluster<cv::Point>::print(void)
{
    for (unsigned int i=0;i<frontier.size();i++)
    {
        cout<<frontier[i].x<<" "<<frontier[i].y<<"; ";
    }
    cout<<endl;

    for (unsigned int i=0;i<rest.size();i++)
    {
        cout<<rest[i].x<<" "<<rest[i].y<<"; ";
    }
    cout<<endl;
}

template <>
void Cluster<cv::Point3i>::print(void)
{
    for (unsigned int i=0;i<frontier.size();i++)
    {
        cout<<frontier[i].x<<" "<<frontier[i].y<<" "<<frontier[i].z<<"; ";
    }
    cout<<endl;

    for (unsigned int i=0;i<rest.size();i++)
    {
        cout<<rest[i].x<<" "<<rest[i].y<<" "<<rest[i].z<<"; ";
    }
    cout<<endl;
}

bool cond(cv::Point a, cv::Point b, int num=0)
{
    return ( (a.x==(b.x+1)|| a.x==b.x || a.x==(b.x-1) ) && ( a.y==(b.y+1) || a.y==b.y || a.y==(b.y-1) ) );
}

bool cond(cv::Point3i a, cv::Point3i b, int num)
{
    int l, u;

    u=incAngle(b.z, num);
    l=decAngle(b.z, num);

    return ( (a.x==(b.x+1)|| a.x==b.x || a.x==(b.x-1) ) && ( a.y==(b.y+1) || a.y==b.y || a.y==(b.y-1) )
             && ( a.z==(u) || a.z==b.z || a.z==(l) ) );
}

template <typename T>
Cluster<T> clustering(Cluster<T>& clust, typename vector<T>::iterator index, int num=0)
{
    if(clust.rest.size()==0)
    {
        return clust;
    }

    T vali=*index;

    clust.rest.erase(index);

    Cluster<T> temp=clust;

    temp.frontier.clear();

    typename vector<T>::iterator it=temp.rest.begin();

    temp.frontier.push_back(vali);

    Cluster<T> tempL;

    bool done=false;

    while(!done)
    {
        done=true;

        while(it!=temp.rest.end())
        {
            bool val=cond(*it,vali, num);
            if( val )
            {

                tempL=clustering(temp,it, num);

                temp.frontier.insert(temp.frontier.end(),tempL.frontier.begin(),tempL.frontier.end());
                temp.rest=tempL.rest;

                it=temp.rest.begin();

                done=false;

                break;
            }
            it++;
        }
    }
    return temp;
}


template <typename T>
vector<vector<T> > cluster_points(vector<T> frontiers, int num=0)
{
    Cluster<T> cl, res;
    cl.frontier.clear();
    cl.rest=frontiers;

    vector<vector<T> > result;
    result.clear();

    while(cl.rest.size()>0)
    {
        res=clustering(cl,cl.rest.begin(), num);

        result.push_back(res.frontier);

        cl=res;
        cl.frontier.clear();

    }

    return result;
}

template <typename T>
vector<T> cluster_points(vector<T>& frontiers, typename vector<T>::iterator index, int num=0)
{
    Cluster<T> cl;
    cl.frontier.clear();
    cl.rest=frontiers;

    typename vector<T>::iterator it=cl.rest.begin();
    typename vector<T>::iterator itf=frontiers.begin();

    while(it!=cl.rest.end())
    {
        if(itf==index)
            break;

        it++;
        itf++;
    }


    cl=clustering(cl,it, num);

    return cl.frontier;
}

vector<cv::Mat> cluster_points(vector<cv::Mat> points, cv::Point3i pos)
{
    vector<cv::Mat> result;

    result.resize(points.size());
    if(points.size()>0)
    {
        for(unsigned int i=0;i<points.size();i++)
        {
            result[i]=cv::Mat(points[0].rows,points[0].cols,CV_8UC1, 0.0);
        }
    }
    else
        result.clear();


    if(pos.z>=(int)points.size())

        return result;
    if(pos.x<0 || pos.x>=points[pos.z].rows || pos.y<0 || pos.y>=points[pos.z].cols)
        return result;


    vector<cv::Point3i> vc;
    vc.clear();
    vc.push_back(pos);

    result[pos.z].at<uchar>(pos.x,pos.y)=255;
    points[pos.z].at<uchar>(pos.x,pos.y)=0;

    while(vc.size()!=0)
    {
        int lx=boundPos(vc[0].x-1,points[vc[0].z].rows);
        int ux=boundPos(vc[0].x+1,points[vc[0].z].rows);
        int ly=boundPos(vc[0].y-1,points[vc[0].z].cols);
        int uy=boundPos(vc[0].y+1,points[vc[0].z].cols);

        for(int i=lx; i<=ux; i++)
        {
            for(int j=ly; j<=uy; j++)
            {
                int la=decAngle(vc[0].z,points.size());
                int ua=incAngle(vc[0].z,points.size());

                if( points[la].at<uchar>(i,j)==255)
                {
                    vc.push_back(cv::Point3i(i,j,la));
                    points[la].at<uchar>(i,j)=0;
                    result[la].at<uchar>(i,j)=255;
                }

                if( points[vc[0].z].at<uchar>(i,j)==255 && (vc[0].x!=i || vc[0].y!=j) )
                {
                    vc.push_back(cv::Point3i(i,j,vc[0].z));
                    points[vc[0].z].at<uchar>(i,j)=0;
                    result[vc[0].z].at<uchar>(i,j)=255;
                }

                if( points[ua].at<uchar>(i,j)==255)
                {
                    vc.push_back(cv::Point3i(i,j,ua));
                    points[ua].at<uchar>(i,j)=0;
                    result[ua].at<uchar>(i,j)=255;
                }
            }
        }

        vc.erase(vc.begin());
    }

    return result;
}

template vector<vector<cv::Point> > cluster_points<cv::Point>(vector<cv::Point> frontiers,  int num=0);
template vector<vector<cv::Point3i> > cluster_points<cv::Point3i>(vector<cv::Point3i> frontiers,  int num=0);
template vector<cv::Point> cluster_points<cv::Point>(vector<cv::Point>& frontiers,vector<cv::Point>::iterator index, int num=0);
template vector<cv::Point3i> cluster_points<cv::Point3i>(vector<cv::Point3i>& frontiers,vector<cv::Point3i>::iterator index,  int num=0);

