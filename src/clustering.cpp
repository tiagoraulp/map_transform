#include "clustering.hpp"

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

    if(b.z==(num-1))
        u=0;
    else
        u=b.z+1;

    if(b.z==0)
        l=num-1;
    else
        l=b.z-1;


    return ( (a.x==(b.x+1)|| a.x==b.x || a.x==(b.x-1) ) && ( a.y==(b.y+1) || a.y==b.y || a.y==(b.y-1) )
             && ( a.z==(u) || a.z==b.z || a.z==(l) ) );
}

template <typename T>
Cluster<T> clustering(Cluster<T>& clust, typename vector<T>::iterator index, int num=0)
{
    if(clust.rest.size()==0)// || index>=clust.rest.size())
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

    cout<<"???"<<endl;
    while(it!=cl.rest.end())
    {
        if(itf==index)
            break;

        it++;
        itf++;
    }


    cout<<"Let+s cluster!!!!"<<endl;

    cl=clustering(cl,it, num);

    return cl.frontier;
}

template vector<vector<cv::Point> > cluster_points<cv::Point>(vector<cv::Point> frontiers,  int num=0);
template vector<vector<cv::Point3i> > cluster_points<cv::Point3i>(vector<cv::Point3i> frontiers,  int num=0);
template vector<cv::Point> cluster_points<cv::Point>(vector<cv::Point>& frontiers,vector<cv::Point>::iterator index, int num=0);
template vector<cv::Point3i> cluster_points<cv::Point3i>(vector<cv::Point3i>& frontiers,vector<cv::Point3i>::iterator index,  int num=0);

