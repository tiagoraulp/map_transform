#include "PAstar.hpp"

#include <queue>
#include <cmath>

#include "ray.hpp"
#include "vector_utils.hpp"

using namespace std;

const int dir=8; // number of possible directions to go at any position
//if dir==4
//static int dx[dir]={1, 0, -1, 0};
//static int dy[dir]={0, 1, 0, -1};
//if dir==8
static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};

template<typename T>
nodePA<T>::nodePA(int xp, int yp, T d, T p, int inf, int def, T ss, float cost1, float cost2, float dist, bool q, bool bfs_, map_transform::VisNode cr_): node<T>(xp, yp, d, p){
    infl=inf;defl=def;sens=ss;opt=dist;crit=cr_;
    power=q;
    bfs=bfs_;
    k1=cost1;
    k2=cost2;
    if(power){
        costEst=&nodePA::costEstimate2;
        costSens=&nodePA::costSensing2;
        K=k1/(2*k2);
    }
    else{
        costEst=&nodePA::costEstimate;
        costSens=&nodePA::costSensing;
    }
}

template<typename T>
float nodePA<T>::costSensing(const int x,const int y) const{
    if( ( (x)*(x)+(y)*(y) )>(defl*defl) )
        return -2;
    else
        return k2*sqrt(x*x+y*y);
}

template<typename T>
float nodePA<T>::costSensing2(const int x,const int y) const{
    if( ( (x)*(x)+(y)*(y) )>(defl*defl) )
        return -2;
    else
        return k2*(x*x+y*y);
}

template<typename T>
float nodePA<T>::costEstimate(const int x,const int y) const
{
    if(k1>k2){
        if( ( (x)*(x)+(y)*(y) )>(defl*defl) )
            return k1*this->costEstimateDist((sqrt(x*x+y*y)-defl), sens)+k2*defl;
        else
            return k2*sqrt(x*x+y*y);
    }
    else{
        if(opt<0)
            return k1*this->costEstimateDist(sqrt(x*x+y*y),sens);
        else{
            float est1;
            if( ( (x)*(x)+(y)*(y) )>=(opt*opt) )
                est1=k1*this->costEstimateDist((sqrt(x*x+y*y)-opt),sens)+k2*opt;
            else
                est1=k2*opt;

            float est2;
            FindMin<float> min_est;
            for(int i=0;i<crit.points.size();i++){
                float xx=crit.points[i].position.x-this->xPos;
                float yy=crit.points[i].position.y-this->yPos;
                float xG=crit.points[i].position.x-Gx;
                float yG=crit.points[i].position.y-Gy;
                min_est.iter(k1*(sqrt(xx*xx+yy*yy)-2*infl)+k2*sqrt(xG*xG+yG*yG));
            }
            est2=min_est.getVal();
            return max(est1,est2);
        }
    }
}

template<typename T>
float nodePA<T>::costEstimate2(const int x,const int y) const{
    if(opt<0){
        if(K<defl)
            if( ( (x)*(x)+(y)*(y) )>=(K*K) )
                return k1*this->costEstimateDist((sqrt(x*x+y*y)-K),sens)+k2*K*K;
            else
                return k2*(x*x+y*y);
        else
            if( ( (x)*(x)+(y)*(y) )>=(defl*defl) )
                return k1*this->costEstimateDist((sqrt(x*x+y*y)-defl),sens)+k2*defl*defl;
            else
                return k2*(x*x+y*y);
    }
    else{
        float est1;
        if(K<defl)
            if( K>opt )
                if( ( (x)*(x)+(y)*(y) )>=(K*K) )
                    est1= k1*this->costEstimateDist((sqrt(x*x+y*y)-K),sens)+k2*K*K;
                else
                    if( ( (x)*(x)+(y)*(y) )>=(opt*opt) )
                        est1= k2*(x*x+y*y);
                    else
                        est1= k2*opt*opt;
            else
                if( ( (x)*(x)+(y)*(y) )>=(opt*opt) )
                    est1= k1*this->costEstimateDist((sqrt(x*x+y*y)-opt),sens)+k2*opt*opt;
                else
                    est1= k2*opt*opt;
        else
            if( ( (x)*(x)+(y)*(y) )>=(defl*defl) )
                est1= k1*this->costEstimateDist((sqrt(x*x+y*y)-defl),sens)+k2*defl*defl;
            else
                if( ( (x)*(x)+(y)*(y) )>=(opt*opt) )
                    est1= k2*(x*x+y*y);
                else
                    est1= k2*opt*opt;
        float est2;
        FindMin<float> min_est;
        for(int i=0;i<crit.points.size();i++){
            float xx=crit.points[i].position.x-this->xPos;
            float yy=crit.points[i].position.y-this->yPos;
            float xG=crit.points[i].position.x-Gx;
            float yG=crit.points[i].position.y-Gy;
            float T2C_dist=sqrt(xG*xG+yG*yG);
            float ext_dist=max(K-T2C_dist,(float)0.0);
            min_est.iter(k1*(sqrt(xx*xx+yy*yy)-2*infl-ext_dist)+k2*(T2C_dist+ext_dist)*(T2C_dist+ext_dist));
        }
        est2=min_est.getVal();
        return max(est1,est2);
    }
}

template<typename T>
T nodePA<T>::getSensing() const {
    return sens;
}

template<typename T>
void nodePA<T>::S2P(void){
     this->priority=sens;
}

template<typename T>
void nodePA<T>::updatePriority(const int & xDest, const int & yDest){
    Gx=xDest;
    Gy=yDest;
    if(!bfs)
     this->priority=this->level+estimate(xDest, yDest)*1;
    else
        this->priority=this->level;
}

template<typename T>
void nodePA<T>::updateSensing(const int & xDest, const int & yDest){
    Gx=xDest;
    Gy=yDest;
    T ss=sensing(xDest, yDest);
    if(ss<0)
        sens=ss;
    else
        sens=this->level+ss;
}

template<typename T>
// Estimation function for the remaining distance to the goal.
const T & nodePA<T>::estimate(const int & xDest, const int & yDest) const{
    static int xd, yd;
    static T d;
    xd=xDest-this->xPos;
    yd=yDest-this->yPos;
    // Euclidian Distance
    d=this->staticCast((this->*costEst)(xd,yd)*10.0);
    // Manhattan distance
    //d=abs(xd)+abs(yd);
    // Chebyshev distance
    //d=max(abs(xd), abs(yd));
    return(d);
}

template<typename T>
const T & nodePA<T>::sensing(const int & xDest, const int & yDest) const{
    int xd, yd;
    static T d;
    xd=xDest-this->xPos;
    yd=yDest-this->yPos;
    d=this->staticCast((this->*costSens)(xd,yd)*10.0);
    return(d);
}

template <typename T>
bool operator<(const nodePA<T> & a, const nodePA<T> & b)
{
  return a.getPriority() > b.getPriority();
}

void PAstar::updateNavMap(std::vector<std::vector<bool> > map){
    map_=map;
}

void PAstar::updateOrMap(cv::Mat map){
    or_map=map.clone();
}

PAstar::PAstar(int in, int de){
    infl=in;
    defl=de;
}

PAstar::PAstar(){
}

template <typename T>
Apath PAstar::run(PointI p0, PointI p1, float k1, float k2, bool quad, float opt, bool bfs, map_transform::VisNode crit){
    Apath path; path.points.clear();path.cost=-1;
    const int n=map_.size();
    if(n<=0){
        //path.points.insert(path.points.begin(),PointI(p0.i,p0.j));
        //path.cost=-2;
        return path;
    }
    const int m=map_[0].size();
    if(m<=0){
        //path.points.insert(path.points.begin(),PointI(p0.i,p0.j));
        //path.cost=-2;
        return path;
    }
    vector<vector<int> > closed_nodes_map;closed_nodes_map.assign(n,vector<int>(m,0));
    vector<vector<int> > open_nodes_map;open_nodes_map.assign(n,vector<int>(m,0));
    vector<vector<int> > dir_map;dir_map.assign(n,vector<int>(m,0));
//    if(opt==-2)
//    {
//        if(msg_rcv[0][p1.i][p1.j] && !msg_rcv[1][p1.i][p1.j])
//        {
//            FindMin<float> crit;
//            bool found=false;

//            for(int x=max(p1.i-infl,0);x<min(p1.i+infl,(int)msg_rcv[0].size());x++)
//            {
//                for(int y=max(p1.j-infl,0);y<min(p1.j+infl,(int)msg_rcv[0][0].size());y++)
//                {
//                    if(msg_rcv[1][p1.i][p1.j])
//                    {
//                        found=true;
//                        float cost=(p1.i-x)*(p1.i-x)+(p1.j-y)*(p1.j-y);
//                        crit.iter(cost);
//                    }
//                }
//            }
//            if(found)
//                opt=sqrt(crit.getVal());
//        }
//    }
    priority_queue<nodePA<T> > pq[3];
    int pqi;
    nodePA<T> n0(p0.i, p0.j, 0, 0, infl, defl, 0, k1, k2, opt, quad, bfs, crit);
    int i, j, x, y, xdx, ydy;
    pqi=0;

    n0.updatePriority(p1.i, p1.j);
    n0.updateSensing(p1.i, p1.j);
    pq[pqi].push(n0);

    //static long int exp_nodes=0, exp_nodes_r=0, tested_goal=0;
    long int exp_nodes=0, exp_nodes_r=0, tested_goal=0;

    while(!pq[pqi].empty() || !pq[2].empty()){
        bool stop=false;
        bool list2=false;
        bool tested=false;
        if(pq[2].size()==0){
            n0=pq[pqi].top();
            x=n0.getxPos(); y=n0.getyPos();
            pq[pqi].pop();
            if(closed_nodes_map[x][y]==1)
                continue;
            if(n0.getSensing()>=0)
                if(n0.getPriority()>=n0.getSensing()){
                    tested_goal++;
                    tested=true;
                    if(isGoal(PointI(x,y),p1))
                        stop=true;
                }
        }
        else{
            if(pq[pqi].size()==0){
                n0=pq[2].top();
                x=n0.getxPos(); y=n0.getyPos();
                pq[2].pop();
                //if(closed_nodes_map[x][y]==1)
                //    continue;
                tested_goal++;
                list2=true;
                if(isGoal(PointI(x,y),p1))
                    stop=true;
            }
            else{
                bool cond=false;
                if(pq[2].top().getSensing()>=0)
                    if(pq[pqi].top().getPriority()>pq[2].top().getSensing())
                    {
                        cond=true;
                    }
                if(!cond){
                    n0=pq[pqi].top();
                    x=n0.getxPos(); y=n0.getyPos();
                    pq[pqi].pop();
                    if(closed_nodes_map[x][y]==1)
                        continue;
                    if(n0.getSensing()>=0)
                        if(n0.getPriority()>=n0.getSensing())
                        {
                            tested_goal++;
                            tested=true;
                            if(isGoal(PointI(x,y),p1))
                                stop=true;
                        }
                }
                else{
                    n0=pq[2].top();
                    x=n0.getxPos(); y=n0.getyPos();
                    pq[2].pop();
                    //if(closed_nodes_map[x][y]==1)
                    //    continue;
                    list2=true;
                    tested_goal++;
                    if(isGoal(PointI(x,y),p1))
                        stop=true;
                }
            }
        }
        if(list2){
            exp_nodes_r++;
        }
        else
            exp_nodes++;

        open_nodes_map[x][y]=0;
        if(stop){
            while(!(x==p0.i && y==p0.j)){
                j=dir_map[x][y];
                path.points.insert(path.points.begin(),PointI(x,y));
                x+=dx[j];
                y+=dy[j];
            }
            path.points.insert(path.points.begin(),PointI(p0.i,p0.j));
            path.cost=(float)n0.getSensing();
            if(opt==-5)
            {
                exp_nodes=0; exp_nodes_r=0, tested_goal=0;
            }
            //myfile[index_file]<<exp_nodes<<"; "<<exp_nodes_r<<"; "<<tested_goal<<"; ";
            //cout<<"Expanded normal nodes: "<<exp_nodes<<endl;
            //cout<<"Expanded backtrack nodes: "<<exp_nodes_r<<endl;
            //cout<<"Goal Tested nodes: "<<tested_goal<<endl;
            return path;
        }
        if(list2)
            continue;
        if(closed_nodes_map[x][y]==1)
            continue;
        closed_nodes_map[x][y]=1;
        for(i=0;i<dir;i++){
            xdx=x+dx[i]; ydy=y+dy[i];
            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 )){
                if(!(!map_[xdx][ydy]
                    || closed_nodes_map[xdx][ydy]==1 //|| ( (dir==1 || dir==3 || dir==5 || dir==7)
                                                     //     && !msg_rcv[r][x+dx[(i-1)%dir]][y+dy[(i-1)%dir]]
                                                     //     && !msg_rcv[r][x+dx[(i+1)%dir]][y+dy[(i+1)%dir]])
                     ))
                {
                    nodePA<T> m0=nodePA<T>( xdx, ydy, n0.getLevel(),
                                 n0.getPriority(), infl, defl, n0.getSensing(), k1, k2, opt, quad, bfs, crit);
                    m0.nextLevel(i);
                    m0.updatePriority(p1.i, p1.j);
                    m0.updateSensing(p1.i, p1.j);
                    if(open_nodes_map[xdx][ydy]==0){
                        open_nodes_map[xdx][ydy]=m0.getPriority();
                        pq[pqi].push(m0);
                        dir_map[xdx][ydy]=(i+dir/2)%dir;
                    }
                    else if(open_nodes_map[xdx][ydy]>m0.getPriority()){
                        open_nodes_map[xdx][ydy]=m0.getPriority();
                        dir_map[xdx][ydy]=(i+dir/2)%dir;
                        pq[pqi].push(m0);
                    }
                }
            }
        }
        if(n0.getSensing()>=0 && !tested){
            n0.S2P();
            pq[2].push(n0);
        }
    }
    path.points.insert(path.points.begin(),PointI(p0.i,p0.j));
    path.cost=-2;
    if(opt==-5)
    {
        exp_nodes=0; exp_nodes_r=0, tested_goal=0;
    }
    //cout<<"Expanded normal nodes: "<<exp_nodes<<endl;
    //cout<<"Expanded backtrack nodes: "<<exp_nodes_r<<endl;
    //cout<<"Goal Tested nodes: "<<tested_goal<<endl;
    //myfile[index_file]<<exp_nodes<<"; "<<exp_nodes_r<<"; "<<tested_goal<<"; ";
    return path;
}

bool PAstar::isGoal(PointI p0, PointI p1){
    if( ( (p0.i-p1.i)*(p0.i-p1.i)+(p0.j-p1.j)*(p0.j-p1.j) )>(defl*defl) )
        return false;
    else{
        if(raytracing(or_map, p0.i, p0.j, p1.i, p1.j, true))
            return true;
        else
            return false;
    }
}

template Apath PAstar::run<float>(PointI p0, PointI p1, float k1=1, float k2=1, bool quad=false, float opt=-3, bool bfs=false, map_transform::VisNode crit=map_transform::VisNode());
template Apath PAstar::run<int>(PointI p0, PointI p1, float k1=1, float k2=1, bool quad=false, float opt=-3, bool bfs=false, map_transform::VisNode crit=map_transform::VisNode());
