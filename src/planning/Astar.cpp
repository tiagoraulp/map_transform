#include "Astar.hpp"

#include <queue>
#include <cmath>

using namespace std;

const int dir=8; // number of possible directions to go at any position
//if dir==4
//static int dx[dir]={1, 0, -1, 0};
//static int dy[dir]={0, 1, 0, -1};
//if dir==8
static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};

bool wrong_direction(int i, int x, int y, cv::Mat dir_nav){
    uchar map_direction=dir_nav.at<uchar>(x,y);
    switch (map_direction) { // from 0 to 7, directions
    case 0: // x positive
        return !( i==6 || i==7 || i==0 || i==1 || i==2 );
        break;
    case 1: // intersection of x positive and y positive
        return !( i==0 || i==1 || i==2 );
        break;
    case 2: // y positive
        return !( i==0 || i==1 || i==2 || i==3 || i==4 );
        break;
    case 3: // intersection of y positive and x negative
        return !( i==2 || i==3 || i==4 );
        break;
    case 4: // x negative
        return !( i==2 || i==3 || i==4 || i==5 || i==6 );
        break;
    case 5: // intersection of x negative and y negative
        return !( i==4 || i==5 || i==6 );
        break;
    case 6: // y negative
        return !( i==4 || i==5 || i==6 || i==7 || i==0 );
        break;
    case 7: // intersection of y negative and x positive
        return !( i==6 || i==7 || i==0 );
        break;
    case 8:
        return false; // free space
        break;
    case 9:
        return true; // obstacle
        break;
    default:
        return true; // obstacle by default
        break;
    }
    return true; //obstacle by default
}

Apath::Apath(){
    cost=-1;
    points.clear();
    exp_nodes=0;
}

template<typename T>
node<T>::node(int xp, int yp, T d, T p){
    xPos=xp; yPos=yp; level=d; priority=p;
}

template<typename T>
int node<T>::getxPos() const {
    return xPos;
}

template<typename T>
int node<T>::getyPos() const {
    return yPos;
}

template<typename T>
T node<T>::getLevel() const {
    return level;
}

template<typename T>
T node<T>::getPriority() const {
    return priority;
}

template<typename T>
bool node<T>::abovePriorityThreshold(T th) const{
    return priority>(th*10);
}

template<typename T>
float node<T>::costEstimate(const int x,const int y) const {
    return costEstimateDist(sqrt(x*x+y*y), level);
}

template<typename T>
const T & node<T>::staticCast(const float d) const {
    static T r;
    r=static_cast<T>(d);
    return r;
}

template<typename T>
void node<T>::updatePriority(const int & xDest, const int & yDest){
    priority=level+estimate(xDest, yDest)*1; //A*
}

template<typename T>
void node<T>::nextLevel(const int & i){ // i: direction
     level+=(dir==8?(i%2==0?10:14.14213562):10);
}

template<typename T>
const T & node<T>::estimate(const int & xDest, const int & yDest) const {
    static int xd, yd;
    static T d;
    xd=xDest-xPos;
    yd=yDest-yPos;
    // Euclidian Distance
    //d=static_cast<int>(costEstimate(xd,yd, infl, defl, opt));
    d=staticCast(costEstimate(xd,yd)*10.0);
    // Manhattan distance
    //d=abs(xd)+abs(yd);
    // Chebyshev distance
    //d=max(abs(xd), abs(yd));
    return d;
}

template <typename T>
bool operator<(const node<T> & a, const node<T> & b)
{
  return a.getPriority() > b.getPriority();
}

template <typename T>
Apath Astar(PointI p0, PointI p1, vector<vector<bool> > msg_rcv, bool disable_thin_diagonals, T th, cv::Mat dir_nav)
{
    Apath path; path.points.clear();path.cost=-1;
    const int n=msg_rcv.size();
    if(n==0)
        return path;
    const int m=msg_rcv[0].size();
    if(m==0)
        return path;
    bool threshold_active;
    if(th<0)
        threshold_active=false;
    else
        threshold_active=true;
    bool dir_nav_active;
    if(dir_nav.rows==0){
        dir_nav=cv::Mat::ones(n,m,CV_8UC1)*8;
        dir_nav_active=false;
    } else {
        dir_nav_active=true;
    }
    vector<vector<int> > closed_nodes_map;closed_nodes_map.assign(n,vector<int>(m,0));
    vector<vector<int> > open_nodes_map;open_nodes_map.assign(n,vector<int>(m,0));
    vector<vector<int> > dir_map;dir_map.assign(n,vector<int>(m,0));
    priority_queue<node<T> > pq;

    node<T> n0(p0.i, p0.j, 0, 0);
    int i, j, x, y, xdx, ydy;
    n0.updatePriority(p1.i, p1.j);
    pq.push(n0);

    while(!pq.empty()){
        n0=pq.top();
        if(threshold_active)
            if(n0.abovePriorityThreshold(th)){
                path.cost=-2;
                return path;
            }
        x=n0.getxPos(); y=n0.getyPos();
        pq.pop();
        open_nodes_map[x][y]=0;
        if(closed_nodes_map[x][y]==1)
            continue;

        closed_nodes_map[x][y]=1;
        if(x==p1.i && y==p1.j){
            path.cost=0;
            while(!(x==p0.i && y==p0.j)){
                j=dir_map[x][y];
                if(j%2==0)
                    path.cost=path.cost+1;
                else
                    path.cost=path.cost+1.41421356237;

                path.points.insert(path.points.begin(),PointI(x,y));
                x+=dx[j];
                y+=dy[j];
            }
            return path;
        }
        for(i=0;i<dir;i++){
            xdx=x+dx[i]; ydy=y+dy[i];
            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || !msg_rcv[xdx][ydy]
                || closed_nodes_map[xdx][ydy]==1 || ( disable_thin_diagonals && (dir==1 || dir==3 || dir==5 || dir==7)
                                                      && !msg_rcv[x+dx[(i-1)%dir]][y+dy[(i-1)%dir]]
                                                      && !msg_rcv[x+dx[(i+1)%dir]][y+dy[(i+1)%dir]])
                || (dir_nav_active && wrong_direction(i,xdx,ydy,dir_nav)) ))
            {
                node<T> m0( xdx, ydy, n0.getLevel(),n0.getPriority());
                m0.nextLevel(i);
                m0.updatePriority(p1.i, p1.j);
                if(open_nodes_map[xdx][ydy]==0){
                    open_nodes_map[xdx][ydy]=m0.getPriority();
                    pq.push(m0);
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0.getPriority()){
                    open_nodes_map[xdx][ydy]=m0.getPriority();
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                    pq.push(m0);
                }
            }
        }
    }
    path.cost=-2;
    return path;
}

template class node<int>;
template class node<float>;

template Apath Astar<float>(PointI p0, PointI p1, vector<vector<bool> > msg, bool disable_thin_diagonals=false, float th=-1, cv::Mat dir_nav=cv::Mat(0,0,CV_8UC1));
template Apath Astar<int>(PointI p0, PointI p1, vector<vector<bool> > msg, bool disable_thin_diagonals=false, int th=-1, cv::Mat dir_nav=cv::Mat(0,0,CV_8UC1));
