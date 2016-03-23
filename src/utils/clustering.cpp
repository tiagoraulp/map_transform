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


ClusterLists::ClusterLists()
{
    cluster.clear();
    extremes.clear();
}

ClusterLists cluster_points(cv::Mat orig, cv::Point pos, cv::Mat map, cv::Mat act)
{
    ClusterLists result;

    if(pos.x<0 || pos.x>=orig.rows || pos.y<0 || pos.y>=orig.cols)
        return result;

    bool method;
    if(map.rows!=orig.rows || map.cols!=orig.cols || act.rows!=orig.rows || act.cols!=orig.cols)
        method=true;
    else
        method=false;

    cv::Mat mem(orig.rows,orig.cols,CV_8UC1, 0.0), points=orig.clone();

    vector<cv::Point> vc;
    vc.clear();
    vc.push_back(pos);

    result.cluster.push_back(pos);

    mem.at<uchar>(pos.x,pos.y)=255;
    points.at<uchar>(pos.x,pos.y)=0;

    while(vc.size()!=0)
    {
        int lx=boundPos(vc[0].x-1,points.rows);
        int ux=boundPos(vc[0].x+1,points.rows);
        int ly=boundPos(vc[0].y-1,points.cols);
        int uy=boundPos(vc[0].y+1,points.cols);

        for(int i=lx; i<=ux; i++)
        {
            for(int j=ly; j<=uy; j++)
            {
                if( points.at<uchar>(i,j)==255 && (vc[0].x!=i || vc[0].y!=j) )
                {
                    vc.push_back(cv::Point(i,j));
                    result.cluster.push_back(cv::Point(i,j));
                    points.at<uchar>(i,j)=0;
                    mem.at<uchar>(i,j)=255;
                }
            }
        }

        vc.erase(vc.begin());
    }

    result.img=mem.clone();

    for(unsigned int k=0; k<result.cluster.size();k++)
    {
        int lx=boundPos(result.cluster[k].x-1,result.img.rows);
        int ux=boundPos(result.cluster[k].x+1,result.img.rows);
        int ly=boundPos(result.cluster[k].y-1,result.img.cols);
        int uy=boundPos(result.cluster[k].y+1,result.img.cols);

        if(method)
        {
            int sum=0;

            for(int i=lx; i<=ux; i++)
            {
                for(int j=ly; j<=uy; j++)
                {
                    if( result.img.at<uchar>(i,j)==255 && (result.cluster[k].x!=i || result.cluster[k].y!=j) )
                    {
                        sum++;
                    }
                }
            }

            if(sum==1)
                result.extremes.push_back(result.cluster[k]);

            //// if(sum==2) test if neighbors are also neighbor points themselves
            /// But this looks difficult and possibly inaccurate
        }
        else
        {
            bool stop=false;

            for(int i=lx; i<=ux; i++)
            {
                for(int j=ly; j<=uy; j++)
                {
                    if( map.at<uchar>(i,j)==0 && (result.cluster[k].x!=i || result.cluster[k].y!=j) )
                    {
                        for(int ii=lx; ii<=ux; ii++)
                        {
                            for(int jj=ly; jj<=uy; jj++)
                            {
                                if( act.at<uchar>(ii,jj)==255 && (result.cluster[k].x!=i || result.cluster[k].y!=j) && (i!=ii || j!=jj) )
                                {
                                    if( max( abs(ii-i),abs(jj-j) )==1 )
                                    {
                                        stop=true;
                                        break;
                                    }
                                }
                            }
                            if(stop)
                                break;
                        }
                        if(stop)
                            break;
                    }
                }
                if(stop)
                    break;
            }

            if(stop)
            {
                result.extremes.push_back(result.cluster[k]);
            }

            //// Possibly more than 2 points if just searching for obstacle
            /// solved with search for actuation point as neighbor. Needs Proof!!!!!!!
        }
    }

    result.rest=points;

    return result;
}

vector<ClusterLists> cluster_points(cv::Mat orig, cv::Mat map, cv::Mat act)
{
    vector<ClusterLists> result;

    for(int i=0;i<orig.rows;i++)
    {
        for(int j=0;j<orig.cols;j++)
        {
            if(orig.at<uchar>(i,j)!=0)
            {
                ClusterLists temp=cluster_points(orig, cv::Point(i,j), map, act);
                orig=temp.rest;
                result.push_back(temp);
            }
        }
    }
    return result;
}

cv::Mat list2Mat(vector<cv::Point> points, cv::Point size)
{
    if(size.x<=0 || size.y<=0)
    {
        return cv::Mat(0,0,CV_8UC1);
    }

    cv::Mat orig=cv::Mat::zeros(size.x, size.y, CV_8UC1);

    for(unsigned int i=0;i<points.size();i++)
    {
        orig.at<uchar>(points[i].x,points[i].y)=255;
    }

    return orig;
}

vector<ClusterLists> cluster_points(vector<cv::Point> points, cv::Point size, cv::Mat map, cv::Mat act)
{
    vector<ClusterLists> result;
    result.clear();

    cv::Mat orig=list2Mat(points,size);
    if(orig.rows==0 || orig.cols==0)
    {
        return result;
    }

    return cluster_points(orig, map, act);
}

ClusterLists cluster_points(vector<cv::Point> points, cv::Point size, cv::Point pos, cv::Mat map, cv::Mat act)
{
    ClusterLists result;

    cv::Mat orig=list2Mat(points,size);
    if(orig.rows==0 || orig.cols==0)
    {
        return result;
    }

    return cluster_points(orig, pos, map, act);
}

vector<cv::Point> extremes_cluster(vector<cv::Point> points)
{
    vector<cv::Point> result;
    result.clear();

    for(unsigned int i=0;i<points.size();i++)
    {
        unsigned int sum=0;

        for(unsigned int j=0;j<points.size();j++)
        {
            if(max(abs(points[i].x-points[j].x),abs(points[i].y-points[j].y))==1)
            {
                sum++;
            }
        }

        if(sum==1)
        {
            result.push_back(points[i]);
        }
    }

    return result;
}

template vector<vector<cv::Point> > cluster_points<cv::Point>(vector<cv::Point> frontiers,  int num=0);
template vector<vector<cv::Point3i> > cluster_points<cv::Point3i>(vector<cv::Point3i> frontiers,  int num=0);
template vector<cv::Point> cluster_points<cv::Point>(vector<cv::Point>& frontiers,vector<cv::Point>::iterator index, int num=0);
template vector<cv::Point3i> cluster_points<cv::Point3i>(vector<cv::Point3i>& frontiers,vector<cv::Point3i>::iterator index,  int num=0);

