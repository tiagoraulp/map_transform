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

PointI::PointI(int a, int b){
    i=a;
    j=b;
}

PointI::PointI(){
    i=0;
    j=0;
}

Apath::Apath(){
    cost=-1;
    points.clear();
}

template<typename T>
class node{
    // current position
    int xPos;
    int yPos;
    // total distance already travelled to reach the node
    T level;
    // priority=level+remaining distance estimate
    T priority;  // smaller: higher priority

    inline float costEstimateDist(const float d, const int ss) const;
    inline float costEstimateDist(const float d, const float ss) const;
    float costEstimate(const int x,const int y) const;
    const T & staticCast(const float d) const;
public:
    node(int xp, int yp, T d, T p){
        xPos=xp; yPos=yp; level=d; priority=p;
    }
    int getxPos() const {return xPos;}
    int getyPos() const {return yPos;}
    T getLevel() const {return level;}
    T getPriority() const {return priority;}

    void updatePriority(const int & xDest, const int & yDest);
    // give better priority to going strait instead of diagonally
    void nextLevel(const int & i);// i: direction
    // Estimation function for the remaining distance to the goal.
    const T & estimate(const int & xDest, const int & yDest) const;
};

template<typename T>
inline float node<T>::costEstimateDist(const float d, const int ss) const {
    return d/14.14213562*14;
}

template<typename T>
inline float node<T>::costEstimateDist(const float d, const float ss) const {
    return d;
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
Apath Astar(PointI p0, PointI p1, vector<vector<bool> > msg_rcv)
{
    Apath path; path.points.clear();path.cost=-1;
    const int n=msg_rcv.size();
    if(n==0)
        return path;
    const int m=msg_rcv[0].size();
    if(m==0)
        return path;
    vector<vector<int> > closed_nodes_map;closed_nodes_map.assign(n,vector<int>(m,0));
    vector<vector<int> > open_nodes_map;open_nodes_map.assign(n,vector<int>(m,0));
    vector<vector<int> > dir_map;dir_map.assign(n,vector<int>(m,0));
    priority_queue<node<T> > pq;

    node<T> n0(p0.i, p0.j, 0, 0);
    int i, j, x, y, xdx, ydy;
    n0.updatePriority(p1.i, p1.j);
    pq.push(n0);

    while(!pq.empty()){
        //n0=node<T>( pq.top().getxPos(), pq.top().getyPos(), pq.top().getLevel(), pq.top().getPriority());
        n0=pq.top();
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
                || closed_nodes_map[xdx][ydy]==1 ))//|| ( (dir==1 || dir==3 || dir==5 || dir==7)
                                                   //   && !msg_rcv[r][x+dx[(i-1)%dir]][y+dy[(i-1)%dir]]
                                                   //   && !msg_rcv[r][x+dx[(i+1)%dir]][y+dy[(i+1)%dir]]) ))
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

template Apath Astar<float>(PointI p0, PointI p1, vector<vector<bool> > msg);
template Apath Astar<int>(PointI p0, PointI p1, vector<vector<bool> > msg);
