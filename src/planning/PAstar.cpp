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

PApath::PApath(): Apath(){
    costM=-1;
    costP=-1;
    exp_nodes_r=0;
    tested_goal=0;
}

template<typename T>
nodePA<T>::nodePA(int xp, int yp, T d, T p, int inf, int def, T ss, float cost2, float dist, bool q, bool bfs_, map_transform::VisNode * cr_, std::vector<float> * dist_crit_goal, bool use_opt_sens, bool use_crit_sens, std::vector<float> * angle_crit_goal, std::vector<float> * angleD_crit_goal): node<T>(xp, yp, d, p){
    infl=inf;defl=def;sens=ss;
    opt=dist-1.5; // opt should be dist in continuous case where dist is minimum; -1.5 (max dist neighbors) adjusts for discretizations errors...
    crit=cr_; dist_c_g=dist_crit_goal; useCritSens=use_crit_sens; useOptSens=use_opt_sens;
    angle_c_g=angle_crit_goal;
    angleD_c_g=angleD_crit_goal;
    power=q;
    bfs=bfs_;
    k2=cost2;
    if(power){
        costEst=&nodePA::costEstimate2;
        costSens=&nodePA::costSensing2;
        K=1.0/(2*k2);
    }
    else{
        costEst=&nodePA::costEstimate;
        costSens=&nodePA::costSensing;
    }
}

template<typename T>
bool nodePA<T>::validSensing(const int x, const int y, float dist) const{
    if( ( dist )>(defl*defl) )
        return false;
    else if( useOptSens && (opt>=0) && (dist<(opt*opt)) )
        return false;
    else if( useCritSens && (opt>=0) && (crit!=NULL) && !(crit->points.empty()) ){
        float angle=atan2(y,x);
        for(int i=0;i<crit->points.size();i++){
            float angleCG;
            if(angle_c_g==NULL || angle_c_g->empty()){
                float xG=Gx-crit->points[i].position.x;
                float yG=Gy-crit->points[i].position.y;
                angleCG=atan2(yG, xG);
            }
            else{
                angleCG=(*angle_c_g)[i];
            }
            float angleDelta;
            if(angleD_c_g==NULL || angleD_c_g->empty()){
                float distCG;
                if(dist_c_g==NULL || dist_c_g->empty()){
                    float xG=Gx-crit->points[i].position.x;
                    float yG=Gy-crit->points[i].position.y;
                    distCG=sqrt(xG*xG+yG*yG);
                }
                else{
                    distCG=(*dist_c_g)[i];
                }
                angleDelta=atan2(infl, distCG);
            }
            else{
                angleDelta=(*angleD_c_g)[i];
            }
            if( abs(boundAngleRN(angleCG-angle))<angleDelta )
                return true;
        }
        return false;
    }
    else
        return true;
}

template<typename T>
float nodePA<T>::costSensing(const int x,const int y) const{
    float dist=(x)*(x)+(y)*(y);
    if(!validSensing(x,y,dist))
        return -2;
    else
        return k2*sqrt(dist);
}

template<typename T>
float nodePA<T>::costSensing2(const int x,const int y) const{
    float dist=(x)*(x)+(y)*(y);
    if(!validSensing(x,y,dist))
        return -2;
    else
        return k2*(dist);
}

template<typename T>
float nodePA<T>::costEstimate(const int x,const int y) const
{
    float dist=(x)*(x)+(y)*(y);
    if(1.0>k2){
        if( ( dist )>(defl*defl) )
            return 1.0*this->costEstimateDist((sqrt(dist)-defl), sens)+k2*defl;
        else
            return k2*sqrt(dist);
        //// could also use opt here?!...
    }
    else{
        if(opt<0)
            return 1.0*this->costEstimateDist(sqrt(dist),sens);
        else{
            float est1;
            if( ( dist )>(opt*opt) )
                est1=1.0*this->costEstimateDist((sqrt(dist)-opt),sens)+k2*opt;
            else
                est1=k2*opt;

            if(crit==NULL || crit->points.empty()){
                return est1;
            }
            else if(crit->points.size()==1){
                if(dist_c_g==NULL || dist_c_g->empty()){
                    float xx=crit->points[0].position.x-this->xPos;
                    float yy=crit->points[0].position.y-this->yPos;
                    float xG=crit->points[0].position.x-Gx;
                    float yG=crit->points[0].position.y-Gy;
                    float est2=1.0*this->costEstimateDist(max(sqrt(xx*xx+yy*yy)-2*infl,(float)0.0),sens)+k2*(sqrt(xG*xG+yG*yG)-1.5);
                    //same correction as before with -1.5; only correcting for xG and yG; do xx and yy also need?
                    return max(est1,est2);
                }
                else{
                    float xx=crit->points[0].position.x-this->xPos;
                    float yy=crit->points[0].position.y-this->yPos;
                    float est2=1.0*this->costEstimateDist(max(sqrt(xx*xx+yy*yy)-2*infl,(float)0.0), sens)+k2*((*dist_c_g)[0]-1.5);
                    return max(est1,est2);
                }
            }
            else{
                if(dist_c_g==NULL || dist_c_g->empty()){
                    float est2;
                    FindMin<float> min_est;
                    for(int i=0;i<crit->points.size();i++){
                        float xx=crit->points[i].position.x-this->xPos;
                        float yy=crit->points[i].position.y-this->yPos;
                        float xG=crit->points[i].position.x-Gx;
                        float yG=crit->points[i].position.y-Gy;
                        min_est.iter(1.0*this->costEstimateDist(max(sqrt(xx*xx+yy*yy)-2*infl,(float)0.0),sens)+k2*(sqrt(xG*xG+yG*yG)-1.5));
                    }
                    est2=min_est.getVal();
                    return max(est1,est2);
                }
                else{
                    float est2;
                    FindMin<float> min_est;
                    for(int i=0;i<crit->points.size();i++){
                        float xx=crit->points[i].position.x-this->xPos;
                        float yy=crit->points[i].position.y-this->yPos;
                        min_est.iter(1.0*this->costEstimateDist(max(sqrt(xx*xx+yy*yy)-2*infl,(float)0.0),sens)+k2*((*dist_c_g)[i]-1.5));
                    }
                    est2=min_est.getVal();
                    return max(est1,est2);
                }
            }
        }
    }
}

template<typename T>
float nodePA<T>::costEstimate2(const int x,const int y) const{
    float dist=(x)*(x)+(y)*(y);
    if(opt<0){
        if(K<defl)
            if( ( dist )>=(K*K) )
                return 1.0*this->costEstimateDist((sqrt(dist)-K),sens)+k2*K*K;
            else
                return k2*(dist);
        else
            if( ( dist )>=(defl*defl) )
                return 1.0*this->costEstimateDist((sqrt(dist)-defl),sens)+k2*defl*defl;
            else
                return k2*(dist);
    }
    else{
        float est1;
        if(K<defl)
            if( K>opt )
                if( ( dist )>=(K*K) )
                    est1= 1.0*this->costEstimateDist((sqrt(dist)-K),sens)+k2*K*K;
                else
                    if( ( dist )>=(opt*opt) )
                        est1= k2*(dist);
                    else
                        est1= k2*opt*opt;
            else
                if( ( dist )>=(opt*opt) )
                    est1= 1.0*this->costEstimateDist((sqrt(dist)-opt),sens)+k2*opt*opt;
                else
                    est1= k2*opt*opt;
        else
            if( ( dist )>=(defl*defl) )
                est1= 1.0*this->costEstimateDist((sqrt(dist)-defl),sens)+k2*defl*defl;
            else
                if( ( dist )>=(opt*opt) )
                    est1= k2*(dist);
                else
                    est1= k2*opt*opt;

        if(crit==NULL || crit->points.empty()){
            return est1;
        }
        else if(crit->points.size()==1){
            if(dist_c_g==NULL || dist_c_g->empty()){
                float xx=crit->points[0].position.x-this->xPos;
                float yy=crit->points[0].position.y-this->yPos;
                float xG=crit->points[0].position.x-Gx;
                float yG=crit->points[0].position.y-Gy;
                float T2C_dist=sqrt(xG*xG+yG*yG)-1.5;
                float ext_dist=max(K-T2C_dist,(float)0.0);
                float est2=1.0*this->costEstimateDist(max(sqrt(xx*xx+yy*yy)-2*infl-ext_dist,(float)0.0), sens)+k2*(T2C_dist)*(T2C_dist);
                return max(est1,est2);
            }
            else{
                float xx=crit->points[0].position.x-this->xPos;
                float yy=crit->points[0].position.y-this->yPos;
                float T2C_dist=(*dist_c_g)[0]-1.5;
                float ext_dist=max(K-T2C_dist,(float)0.0);
                float est2=1.0*this->costEstimateDist(max(sqrt(xx*xx+yy*yy)-2*infl-ext_dist,(float)0.0), sens)+k2*(T2C_dist)*(T2C_dist);
                //cout<<"TEST"<<endl;
                return max(est1,est2);
            }
        }
        else{
            if(dist_c_g==NULL || dist_c_g->empty()){
                float est2;
                FindMin<float> min_est;
                for(int i=0;i<crit->points.size();i++){
                    float xx=crit->points[i].position.x-this->xPos;
                    float yy=crit->points[i].position.y-this->yPos;
                    float xG=crit->points[i].position.x-Gx;
                    float yG=crit->points[i].position.y-Gy;
                    float T2C_dist=sqrt(xG*xG+yG*yG)-1.5;
                    if(this->xPos==180 && this->yPos==180)
                        cout<<"T2C: "<<T2C_dist<<endl;
                    float ext_dist=max(K-T2C_dist,(float)0.0);
                    //min_est.iter(1.0*(sqrt(xx*xx+yy*yy)-2*infl-ext_dist)+k2*(T2C_dist+ext_dist)*(T2C_dist+ext_dist));
                    // can only guarantee the sensing distance is T2C_dist...
                    min_est.iter(1.0*this->costEstimateDist(max(sqrt(xx*xx+yy*yy)-2*infl-ext_dist,(float)0.0), sens)+k2*(T2C_dist)*(T2C_dist));
                }
                est2=min_est.getVal();
                return max(est1,est2);
            }
            else{
                float est2;
                FindMin<float> min_est;
                for(int i=0;i<crit->points.size();i++){
                    float xx=crit->points[i].position.x-this->xPos;
                    float yy=crit->points[i].position.y-this->yPos;
                    float T2C_dist=(*dist_c_g)[i]-1.5;
                    float ext_dist=max(K-T2C_dist,(float)0.0);
                    //min_est.iter(1.0*(sqrt(xx*xx+yy*yy)-2*infl-ext_dist)+k2*(T2C_dist+ext_dist)*(T2C_dist+ext_dist));
                    // can only guarantee the sensing distance is T2C_dist...
                    min_est.iter(1.0*this->costEstimateDist(max(sqrt(xx*xx+yy*yy)-2*infl-ext_dist,(float)0.0), sens)+k2*(T2C_dist)*(T2C_dist));
                }
                est2=min_est.getVal();
                //cout<<"TEST1234"<<endl;
                return max(est1,est2);
            }
        }
    }
}

template<typename T>
T nodePA<T>::getSensing() const {
    return sens;
}

template<typename T>
double nodePA<T>::getCost() const {
    return ((double)getSensing())/10.0;
}

template<typename T>
double nodePA<T>::getMotionCost() const {
    return ((double)(this->level))/10.0;
}

template<typename T>
double nodePA<T>::getSensingCost() const {
    return ((double)sensing(Gx, Gy))/10.0;
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
    //if(this->xPos==180 && this->yPos==180)
    //    cout<<"G: "<<this->priority<<endl;
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

void PAstar::resetExpansion(void){
    expansion=cv::Mat::zeros(or_map.rows,or_map.cols, CV_8UC1);
}

cv::Mat PAstar::getExpansion(void){
    return expansion;
}

PAstar::PAstar(int in, int de){
    infl=in;
    defl=de;
}

PAstar::PAstar(){
}

template <typename T>
PApath PAstar::run(PointI p0, PointI p1, float k2, bool quad, float opt, bool bfs, map_transform::VisNode * crit, std::vector<float> * dist_crit_goal, bool use_opt_sens, bool use_crit_sens, std::vector<float> * angle_crit_goal, std::vector<float> * angleD_crit_goal){
    PApath path; path.points.clear();path.cost=-1;
    const int n=map_.size();
    resetExpansion();
    //cout<<"TEST!!!!!!!!!!!!!"<<endl;
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
    nodePA<T> n0(p0.i, p0.j, 0, 0, infl, defl, 0, k2, opt, quad, bfs, crit, dist_crit_goal, use_opt_sens, use_crit_sens, angle_crit_goal, angleD_crit_goal);
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
            //cout<<"ER1!!"<<endl;
            expansion.at<uchar>(x,y)=255;
            //cout<<"ER2!!"<<endl;
        }
        else
            exp_nodes++;

        //if(!crit.points.empty())
        //    cout<<"Hmmm: "<<x<<"; "<<y<<endl;

        open_nodes_map[x][y]=0;
        if(stop){
            //cout<<"STOP1!!"<<endl;
            expansion.at<uchar>(x,y)=255;
            //cout<<"STOP2!!"<<endl;
            while(!(x==p0.i && y==p0.j)){
                j=dir_map[x][y];
                path.points.insert(path.points.begin(),PointI(x,y));
                x+=dx[j];
                y+=dy[j];
            }
            path.points.insert(path.points.begin(),PointI(p0.i,p0.j));
            path.cost=n0.getCost();
            path.costM=n0.getMotionCost();
            path.costP=n0.getSensingCost();
            //cout<<"Solution M: "<<path.costM<<"; P: "<<path.costP<<endl;
            if(opt==-5)
            {
                exp_nodes=0; exp_nodes_r=0, tested_goal=0;
            }
            path.exp_nodes=exp_nodes;
            path.exp_nodes_r=exp_nodes_r;
            path.tested_goal=tested_goal;
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
                                 n0.getPriority(), infl, defl, n0.getSensing(), k2, opt, quad, bfs, crit, dist_crit_goal, use_opt_sens, use_crit_sens, angle_crit_goal, angleD_crit_goal);
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
                    //cout<<"OPEN1!!"<<endl;
                    expansion.at<uchar>(xdx,ydy)=100;
                    //cout<<"OPEN2!!"<<endl;
                }
            }
        }
        if(n0.getSensing()>=0 && !tested){
            n0.S2P();
            pq[2].push(n0);
            //cout<<"CLOSE1!!"<<endl;
            expansion.at<uchar>(x,y)=200;
            //cout<<"CLOSE2!!"<<endl;
        }
        else if(n0.getSensing()>=0){
            //cout<<"SER1!!"<<endl;
            expansion.at<uchar>(x,y)=255;
            //cout<<"SER2!!"<<endl;
        }
    }
    path.points.insert(path.points.begin(),PointI(p0.i,p0.j));
    path.cost=-2;
    path.costM=-2;
    path.costP=-2;
    if(opt==-5)
    {
        exp_nodes=0; exp_nodes_r=0, tested_goal=0;
    }
    path.exp_nodes=exp_nodes;
    path.exp_nodes_r=exp_nodes_r;
    path.tested_goal=tested_goal;
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

template PApath PAstar::run<float>(PointI p0, PointI p1, float k2=1, bool quad=false, float opt=-3, bool bfs=false, map_transform::VisNode * crit=NULL, std::vector<float> * dist_crit_goal=NULL, bool use_opt_sens=false, bool use_crit_sens=false, std::vector<float> * angle_crit_goal=NULL, std::vector<float> * angleD_crit_goal=NULL);
template PApath PAstar::run<int>(PointI p0, PointI p1, float k2=1, bool quad=false, float opt=-3, bool bfs=false, map_transform::VisNode * crit=NULL, std::vector<float> * dist_crit_goal=NULL, bool use_opt_sens=false, bool use_crit_sens=false, std::vector<float> * angle_crit_goal=NULL, std::vector<float> * angleD_crit_goal=NULL);
