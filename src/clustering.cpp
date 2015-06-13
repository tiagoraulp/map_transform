#include "clustering.hpp"

Cluster::Cluster()
{
    frontier.clear();
    rest.clear();
}

void Cluster::append(Cluster b)
{
    frontier.insert(frontier.end(),b.frontier.begin(),b.frontier.end());
    rest.insert(rest.end(),b.rest.begin(),b.rest.end());
}

void Cluster::print(void)
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

Cluster clustering(Cluster clust, unsigned int index)
{

    clust.frontier.clear();

    if(clust.rest.size()==0 || index>=clust.rest.size())
    {
        return clust;
    }


    Cluster temp=clust;
    temp.rest.erase(temp.rest.begin()+index);
    temp.frontier.push_back(clust.rest[index]);

    Cluster tempL;

    bool done=false;

    while(!done)
    {
        done=true;

        for(unsigned int i=0;i<temp.rest.size();i++)
        {
            if(  (temp.rest[i].x==(clust.rest[index].x+1)|| temp.rest[i].x==clust.rest[index].x || temp.rest[i].x==(clust.rest[index].x-1) ) && ( temp.rest[i].y==(clust.rest[index].y+1) || temp.rest[i].y==clust.rest[index].y || temp.rest[i].y==(clust.rest[index].y-1) ) )
            {

                tempL=clustering(temp,i);

                temp.frontier.insert(temp.frontier.end(),tempL.frontier.begin(),tempL.frontier.end());
                temp.rest=tempL.rest;

                done=false;

                break;

            }

        }
    }
    return temp;
}

vector<vector<cv::Point> > cluster_points(vector<cv::Point> frontiers)
{
    Cluster cl, res;
    cl.frontier.clear();
    cl.rest=frontiers;

    vector<vector<cv::Point> > result;
    result.clear();

    while(cl.rest.size()>0)
    {
        res=clustering(cl,0);

        result.push_back(res.frontier);

        cl=res;
        cl.frontier.clear();
    }

    return result;
}

