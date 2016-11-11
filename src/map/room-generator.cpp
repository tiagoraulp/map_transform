#include "ros/ros.h"
#include "ros/package.h"
#include <signal.h>
#include <fstream>
#include <opencv2/core/core.hpp>
#include "std_srvs/Empty.h"
#include <queue>
#include <cmath>

using namespace std;

class RoomGen{
private:
    ros::NodeHandle nh_;
    float res;
    int width,height;
    void process(ofstream& map);
    ros::ServiceClient permission;
public:
    RoomGen(ros::NodeHandle nh): nh_(nh){
        srand (time(NULL));
        permission = nh_.serviceClient<std_srvs::Empty>("generateMap");
    }
    bool run(void);
    bool finish(void);
};

class Door{
private:
    float pos;
    float size;
    float doorMin;
    float doorMax;
    float wall;
    float doorN, doorP;
    void generate(){
        pos=size/2.0+((float)rand()/RAND_MAX)*(wall-2.0*size/2.0);
        doorN=pos-size/2.0;
        doorP=pos+size/2.0;
        if(doorN<0) //unnecessary from here since pos is calculated using size
            doorN=0;
        if(doorP>wall)
            doorP=wall;
        pos=(doorN+doorP)/2.0;
        size=(doorP-doorN);
    }
    void generate(float max_pos){
        //pos=size/2.0+((float)rand()/RAND_MAX)*(min(wall-size/2.0,max_pos)-size/2.0);
        //// TODO: generate random pos so there is random space between rooms!
        pos=max_pos;
        doorN=pos-size/2.0;
        doorP=pos+size/2.0;
        if(doorN<0) //unnecessary from here since pos is calculated using size
            doorN=0;
        if(doorP>wall)
            doorP=wall;
        pos=(doorN+doorP)/2.0;
        size=(doorP-doorN);
    }
public:
    Door(float dmin, float dmax, float wall_): doorMin(dmin), doorMax(dmax), wall(wall_){
        size=doorMin+((float)rand()/RAND_MAX)*(doorMax-doorMin);
        generate();
    }
    Door(){}
    float getSize(){
        return size;
    }
    void invert(void){
        pos=size-pos;
        float posNtemp=doorN;
        doorN=size-doorP;
        doorP=size-posNtemp;
    }
    float getWallSize(){
        return wall;
    }
    float getN(){
        return doorN;
    }
    float getP(){
        return doorP;
    }
    void update(float wall_size){
        wall=wall_size;
        generate();
    }
    void updateIWS(float wall_size){
        wall=wall_size;
    }
    void updateMP(float wall_size, float max_pos){
        wall=wall_size;
        generate(max_pos);
    }
    void update(float wall_size, float posN_){
        wall=wall_size;
        doorN=posN_;
        doorP=doorN+size;
        if(doorN<0)
            doorN=0;
        if(doorP>wall)
            doorP=wall;
        pos=(doorN+doorP)/2.0;
        size=(doorP-doorN);
    }
};

enum Dir{Hor, Ver};
enum Sign{Pos, Neg};

Dir neg(Dir dir_){
    if(dir_==Hor)
        return Ver;
    else
        return Hor;
}

Sign neg(Sign sign_){
    if(sign_==Pos)
        return Neg;
    else
        return Pos;
}

struct Limits{
    float xmin, xmax, ymin, ymax;
    Limits(float xmin_,float xmax_,float ymin_,float ymax_): xmin(xmin_), xmax(xmax_), ymin(ymin_), ymax(ymax_){
    }
    Limits(): xmin(0), xmax(0), ymin(0), ymax(0){
    }
    bool inside(float x, float y){
        if( x>=xmin && x<=xmax && y>=ymin && y<=ymax )
            return true;
        else
            return false;
    }
    bool inside(cv::Point2f pt){
        return inside(pt.x, pt.y);
    }
    void fix(float& x,float& y){
        if(x<xmin)
            x=xmin;
        else if (x>xmax)
            x=xmax;

        if (y<ymin)
            y=ymin;
        else if (y>ymax)
            y=ymax;
    }
    void fix(cv::Point2f& pt){
        fix(pt.x, pt.y);
    }
    float diff(float x,float y){
        if(x<xmin)
            return xmin-x;
        else if (x>xmax)
            return x-xmax;

        if (y<ymin)
            return ymin-y;
        else if (y>ymax)
            return y-ymax;

        return -1;
    }
    bool diff(cv::Point2f pt){
        return diff(pt.x, pt.y);
    }
    void left(float& x,float& y){
        x=min(x-xmin,xmax-x);
        y=min(y-ymin,ymax-y);
    }
    void left(float& x,float& y, float direction){
        if(direction>0){
            x=xmax-x;
            y=ymax-y;
        }
        else{
            x=x-xmin;
            y=y-ymin;
        }
    }
    void left(cv::Point2f& pt){
        left(pt.x, pt.y);
    }
    void left(cv::Point2f& pt, float direction){
        left(pt.x, pt.y, direction);
    }
    void leftAbs(float& x,float& y){
        x=min(abs(x-xmin),abs(xmax-x));
        y=min(abs(y-ymin),abs(ymax-y));
    }
    void leftAbs(cv::Point2f& pt){
        leftAbs(pt.x, pt.y);
    }
    bool border(float x, float y, float th){
        if( x<=(xmin+th) || x>=(xmax-th) || y<=(ymin+th) || y>=(ymax-th) )
            return true;
        else
            return false;
    }
    bool border(cv::Point2f pt, float th){
        return border(pt.x, pt.y, th);
    }
    int closer_to_border(cv::Point2f pt0, cv::Point2f pt1, Dir dir){
        if(dir==Hor)
            return closer_to_border(pt0.y, pt1.y, ymin, ymax);
        else
            return closer_to_border(pt0.x, pt1.x, xmin, xmax);
    }
    int closer_to_border(float z0, float z1, float zmin, float zmax){
        float diff0=min(z0-zmin, zmax-z0), diff1=min(z1-zmin, zmax-z1);
        if(diff0<diff1)
            return 0;
        else
            return 1;
    }
};

struct growing_direction{
    Dir dir;
    Sign sign;
    growing_direction(Dir dir_, Sign sign_): dir(dir_), sign(sign_){
    }
    growing_direction(){
    }
};

typedef growing_direction GD;

class Wall{
private:
    static unsigned int nID;
    unsigned int id;
    vector<cv::Point2f> pt;
    Dir dir;
    float sign;
    float wall_size;
    float wallMin;
    float wallMax;
    bool hasDoor;
    Door door;
    float doorMin;
    float doorMax;
    Limits limits;
    unsigned int roomID;
    GD gd;
    void invert(void){
        sign*=-1;
        vector<cv::Point2f> pt_temp=pt;
        for(unsigned int i=0; i<pt.size(); i++){
            if(i<2)
                pt[i]=pt_temp[1-i];
            else
                pt[i]=pt_temp[5-i];

        }
        door.invert();
    }
    void updatePosition(int index, float size){
        if(dir==Hor)
            pt[index].y+=sign*size;
        else
            pt[index].x+=sign*size;
    }
    float chooseDir(float x, float y){
        if(dir==Hor)
            return y;
        else
            return x;
    }
    float chooseDir(cv::Point2f pt){
        return chooseDir(pt.x,pt.y);
    }
    float chooseConstDir(float x, float y){
        if(dir==Hor)
            return x;
        else
            return y;
    }
    float chooseConstDir(cv::Point2f pt){
        return chooseConstDir(pt.x,pt.y);
    }
    void correct_limits(int ind){
        if(!limits.inside(pt[ind].x,pt[ind].y)){
            wall_size-=limits.diff(pt[ind].x,pt[ind].y)<0?0:limits.diff(pt[ind].x,pt[ind].y);
            limits.fix(pt[ind].x,pt[ind].y);
        }
        else{
            float xleft=pt[ind].x, yleft=pt[ind].y;
            float direction;
            if( (ind==0 && sign>0) || (ind==1 && sign<0))
                direction=-1.0;
            else
                direction=1.0;
            limits.left(xleft, yleft, direction);
            float diff=chooseDir(xleft,yleft);
            if(diff>=wallMin){
                if(wall_size<wallMin){
                    float sign_;
                    if(ind==1)
                        sign_=1;
                    else
                        sign_=-1;
                    float margin=wallMin-wall_size;
                    wall_size+=margin;
                    updatePosition(ind,sign_*margin);
                }
                if(wall_size>wallMax){
                    float adj_diff, sign_;
                    if(ind==1){
                        sign_=1.0;
                        adj_diff=door.getWallSize()-door.getP();
                    }else{
                        sign_=-1.0;
                        adj_diff=door.getN();
                    }
                    if((wall_size-wallMax)>adj_diff){
                        cout<<"Error Max size wall, impossible to reduce to garantee max limit; ";
                        cout<<wall_size<<"; "<<wallMax<<endl;
                    }
                    else{
                        float diff_margin=(wall_size-wallMax);
                        wall_size-=diff_margin;
                        updatePosition(ind,-sign_*diff_margin);
                    }

                }
            }
            else{
                float sign_;
                if(ind==1)
                    sign_=1;
                else
                    sign_=-1;
                if(!has_door()){
                    if( (wall_size-(wallMin-diff))<wallMin ){
                        wall_size+=diff;
                        updatePosition(ind,sign_*diff);
                    }
                    else{
                        float margin=((float)rand()/RAND_MAX)*(wall_size+diff-2*wallMin);
                        float diff_margin=wallMin+margin-wall_size;
                        wall_size+=diff_margin;
                        updatePosition(ind,sign_*diff_margin);
                    }
                }
                else{
                    float adj_diff;
                    if(ind==1){
                        adj_diff=door.getWallSize()-door.getP();
                    }else{
                        adj_diff=door.getN();
                    }
                    if( (wallMin-diff)>adj_diff){
                        wall_size+=diff;
                        updatePosition(ind,sign_*diff);
                    }
                    else{
                        float margin=((float)rand()/RAND_MAX)*(adj_diff-(wallMin-diff));
                        float diff_margin=(wallMin-diff)+margin;
                        wall_size-=diff_margin;
                        updatePosition(ind,-sign_*diff_margin);
                    }
                }
            }
        }
    }
    void generate(){
        wall_size=wallMin+((float)rand()/RAND_MAX)*(wallMax-wallMin);
        updatePosition(1, wall_size);
        correct_limits(1);
        if(!limits.inside(getInit().x,getInit().y)){
            cout<<"Error Generate init: ";
            cout<<getInit().x<<" "<<getInit().y<<endl;
        }
        if(!limits.inside(getEnd().x,getEnd().y)){
            cout<<"Error Generate end: ";
            cout<<getEnd().x<<" "<<getEnd().y<<endl;
        }
        if(wall_size<wallMin){
            cout<<"Error Generate wallmin: ";
            cout<<wall_size<<"=size which is < minsize="<<wallMin<<endl;
        }
        if(wall_size>wallMax){
            cout<<"Error Generate wallmax: ";
            cout<<wall_size<<"=size which is > maxsize="<<wallMax<<endl;
        }
        if((wall_size-(abs(getEnd().x-getInit().x)+abs(getEnd().y-getInit().y)))>0.01){
            cout<<"Error Generate wall size diff pts: ";
            cout<<wall_size<<"; "<<getInit().x<<" "<<getInit().y<<"; "<<getEnd().x<<" "<<getEnd().y<<endl;
        }
        if( (chooseDir(pt[2]-pt[0])*sign)<0 || (chooseDir(pt[3]-pt[2])*sign)<0 || (chooseDir(pt[1]-pt[3])*sign)<0 ){
            cout<<"Error Correct pt order: ";
            cout<<getInit().x<<" "<<getInit().y<<"; "<<pt[2].x<<" "<<pt[2].y<<"; "<<pt[3].x<<" "<<pt[3].y<<"; "<<getEnd().x<<" "<<getEnd().y<<endl;
        }
    }
public:
    Wall(cv::Point2f p0_, Dir dir_, Sign sign_, float wmin, float wmax, float dmin, float dmax, Limits limits_, unsigned int roomID_, GD gd_):
            id(nID++), dir(dir_), wallMin(wmin), wallMax(wmax), doorMin(dmin), doorMax(dmax), limits(limits_), roomID(roomID_), gd(gd_) {
        if(sign_==Pos)
            sign=1.0;
        else
            sign=-1.0;
        pt.assign(4,p0_);
        hasDoor=false;
        generate();
        if( limits.border(getInit(), wallMin/2.0) && limits.border(getEnd(), wallMin/2.0) )
            return;
        createDoor(doorMin, doorMax);
        if( ((wall_size-door.getWallSize())>0.01) || (abs(abs(chooseDir(pt[3]-pt[2]))-door.getSize())>0.01) || (abs(abs(chooseDir(pt[2]-pt[0]))-door.getN())>0.01) || (abs(abs(chooseDir(pt[1]-pt[3]))-abs(door.getWallSize()-door.getP()))>0.01) ){
            cout<<"Error Correct door wall integrity: Wall: ";
            cout<<getInit().x<<" "<<getInit().y<<"; "<<pt[2].x<<" "<<pt[2].y<<"; "<<pt[3].x<<" "<<pt[3].y<<"; "<<getEnd().x<<" "<<getEnd().y<<"; ";
            cout<<"Door: "<<door.getWallSize()<<"; "<<door.getSize()<<" "<<door.getN()<<"; "<<door.getP()<<endl;
        }
        if( (abs(door.getSize()-abs(door.getP()-door.getN()))>0.01) || (door.getSize()>doorMax) || (door.getSize()<doorMin) ){
            cout<<"Error Door integrity: Door limits: ";
            cout<<doorMin<<"; "<<doorMax<<"; ";
            cout<<"Door: "<<door.getWallSize()<<"; "<<door.getSize()<<" "<<door.getN()<<"; "<<door.getP()<<endl;
        }
    }
    Wall(){
        hasDoor=false;
    }
    void update(cv::Point2f p){
        pt.assign(4,p);
        id=nID++;
        updatePosition(1, wall_size);
        hasDoor=false;
        if( limits.border(getInit(), wallMin/2.0) && limits.border(getEnd(), wallMin/2.0) )
            return;
        createDoor(doorMin, doorMax);
        if( ((wall_size-door.getWallSize())>0.01) || (abs(abs(chooseDir(pt[3]-pt[2]))-door.getSize())>0.01) || (abs(abs(chooseDir(pt[2]-pt[0]))-door.getN())>0.01) || (abs(abs(chooseDir(pt[1]-pt[3]))-abs(door.getWallSize()-door.getP()))>0.01) ){
            cout<<"Error Correct door wall integrity: Wall: ";
            cout<<getInit().x<<" "<<getInit().y<<"; "<<pt[2].x<<" "<<pt[2].y<<"; "<<pt[3].x<<" "<<pt[3].y<<"; "<<getEnd().x<<" "<<getEnd().y<<"; ";
            cout<<"Door: "<<door.getWallSize()<<"; "<<door.getSize()<<" "<<door.getN()<<"; "<<door.getP()<<endl;
        }
        if( (abs(door.getSize()-abs(door.getP()-door.getN()))>0.01) || (door.getSize()>doorMax) || (door.getSize()<doorMin) ){
            cout<<"Error Door integrity: Door limits: ";
            cout<<doorMin<<"; "<<doorMax<<"; ";
            cout<<"Door: "<<door.getWallSize()<<"; "<<door.getSize()<<" "<<door.getN()<<"; "<<door.getP()<<endl;
        }
    }
    bool updateFromDoor(void){
        if(!hasDoor)
            return false;
        id=nID++;
        wall_size=max(wallMin,door.getSize())+((float)rand()/RAND_MAX)*(wallMax-max(wallMin,door.getSize()));
        Door prev_door=door;
        door.update(wall_size);
        updatePosition(0,prev_door.getN()-door.getN());
        updatePosition(1,-(prev_door.getWallSize()-prev_door.getP())+(door.getWallSize()-door.getP()));
        int ind=limits.closer_to_border(getInit(),getEnd(), dir);
        correct_limits(ind);
        door.update(wall_size, abs(chooseDir(pt[0]-pt[2])));
        correct_limits(1-ind);
        door.update(wall_size, abs(chooseDir(pt[0]-pt[2])));
        if(!limits.inside(getInit().x,getInit().y)){
            cout<<"Error from Door Init: ";
            cout<<getInit().x<<" "<<getInit().y<<endl;
        }
        if(!limits.inside(getEnd().x,getEnd().y)){
            cout<<"Error from Door End: ";
            cout<<getEnd().x<<" "<<getEnd().y<<endl;
        }
        if(wall_size<wallMin){
            cout<<"Error Correct wallmin: ";
            cout<<wall_size<<"=size which is < minsize="<<wallMin<<endl;
        }
        if(wall_size>wallMax){
            cout<<"Error Correct wallmax: ";
            cout<<wall_size<<"=size which is > maxsize="<<wallMax<<endl;
        }
        if((wall_size-(abs(getEnd().x-getInit().x)+abs(getEnd().y-getInit().y)))>0.01){
            cout<<"Error Correct wall size diff pts: ";
            cout<<wall_size<<"; "<<getInit().x<<" "<<getInit().y<<"; "<<getEnd().x<<" "<<getEnd().y<<endl;
        }
        if( (chooseDir(pt[2]-pt[0])*sign)<0 || (chooseDir(pt[3]-pt[2])*sign)<0 || (chooseDir(pt[1]-pt[3])*sign)<0 ){
            cout<<"Error Correct pt order: ";
            cout<<getInit().x<<" "<<getInit().y<<"; "<<pt[2].x<<" "<<pt[2].y<<"; "<<pt[3].x<<" "<<pt[3].y<<"; "<<getEnd().x<<" "<<getEnd().y<<endl;
        }
        if( ((wall_size-door.getWallSize())>0.01) || (abs(abs(chooseDir(pt[3]-pt[2]))-door.getSize())>0.01) || (abs(abs(chooseDir(pt[2]-pt[0]))-door.getN())>0.01) || (abs(abs(chooseDir(pt[1]-pt[3]))-abs(door.getWallSize()-door.getP()))>0.01) ){
            cout<<"Error Correct door wall integrity: Wall: ";
            cout<<getInit().x<<" "<<getInit().y<<"; "<<pt[2].x<<" "<<pt[2].y<<"; "<<pt[3].x<<" "<<pt[3].y<<"; "<<getEnd().x<<" "<<getEnd().y<<"; ";
            cout<<"Door: "<<door.getWallSize()<<"; "<<door.getSize()<<" "<<door.getN()<<"; "<<door.getP()<<endl;
        }
        if( (abs(door.getSize()-abs(door.getP()-door.getN()))>0.01) || (door.getSize()>doorMax) || (door.getSize()<doorMin) ){
            cout<<"Error Door integrity: Door limits: ";
            cout<<doorMin<<"; "<<doorMax<<"; ";
            cout<<"Door: "<<door.getWallSize()<<"; "<<door.getSize()<<" "<<door.getN()<<"; "<<door.getP()<<endl;
        }
        return true;
    }
    bool updateFromDoor(Wall other){
        if(!hasDoor)
            return false;
        id=nID++;

        if( ((other.getConstPos()<chooseDir(pt[2])) && sign<0) || ((other.getConstPos()>chooseDir(pt[2])) && sign>0) )
            invert();

        cv::Point2f temp=pt[3];
        limits.left(temp, sign);
        float dist=chooseDir(temp);

        float maxPos=abs(chooseDir(pt[2])-other.getConstPos());
        float doorN_Pos=((float)rand()/RAND_MAX)*(maxPos);
        float marginNI=maxPos-doorN_Pos;
        float marginND=doorN_Pos;
        float minSize=doorN_Pos+door.getSize();
        float maxSize=min(wallMax,minSize+dist);
        float wSize=minSize+((float)rand()/RAND_MAX)*(maxSize-minSize);
        float marginPD=wSize-minSize;
        float marginPI=minSize+dist-wSize;

        Door prev_door=door;
        door.update(wSize, doorN_Pos);
        wall_size=wSize;

        updatePosition(0,prev_door.getN()-door.getN());
        updatePosition(1,-(prev_door.getWallSize()-prev_door.getP())+(door.getWallSize()-door.getP()));

        if(marginPI<wallMin){
            if(wSize<wallMin){
                wSize=wall_size+marginPI;
                Door prev_door=door;
                door.updateIWS(wSize);
                wall_size=wSize;
                updatePosition(1,-(prev_door.getWallSize()-prev_door.getP())+(door.getWallSize()-door.getP()));

                float distMiss=(wallMin-wSize);
                if(marginNI>=distMiss){
                    wSize=wall_size+distMiss;
                    doorN_Pos+=distMiss;
                    Door prev_door=door;
                    door.update(wSize, doorN_Pos);
                    wall_size=wSize;
                    updatePosition(0,prev_door.getN()-door.getN());
                    updatePosition(1,-(prev_door.getWallSize()-prev_door.getP())+(door.getWallSize()-door.getP()));
                }
                else{
                    return false;
                }
            }
            else{
                // 3 techniques
                float wSI=wSize+marginPI-marginND;
                float change=wallMin-marginPI;
                float wSDMax=wSize-change+marginNI;
                float wSDMin=wSize-change;
                bool methodI=false;
                if( (wSI<=wallMax) && (change<=marginPD && wSDMax>=wallMin && wSDMin<=wallMax) )
                {
                    if( rand()<=0.5 )
                        methodI=true;
                    else
                        methodI=false;
                }
                else if( wSI<=wallMax )
                    methodI=true;
                else if( change<=marginPD && wSDMax>=wallMin && wSDMin<=wallMax )
                    methodI=false;
                else
                    return false;

                if(methodI){
                    wSize=wall_size+marginPI;
                    Door prev_door=door;
                    door.updateIWS(wSize);
                    wall_size=wSize;
                    updatePosition(1,-(prev_door.getWallSize()-prev_door.getP())+(door.getWallSize()-door.getP()));

                    float distMiss=wSize-wallMax;
                    if(distMiss>0){
                        wSize=wall_size-distMiss;
                        doorN_Pos-=distMiss;
                        Door prev_door=door;
                        door.update(wSize, doorN_Pos);
                        wall_size=wSize;
                        updatePosition(0,prev_door.getN()-door.getN());
                        updatePosition(1,-(prev_door.getWallSize()-prev_door.getP())+(door.getWallSize()-door.getP()));
                    }
                }
                else{
                    wSize=wall_size-change;
                    Door prev_door=door;
                    door.updateIWS(wSize);
                    wall_size=wSize;
                    updatePosition(1,-(prev_door.getWallSize()-prev_door.getP())+(door.getWallSize()-door.getP()));

                    float distMiss=wallMin-wSize;
                    if(distMiss>0){
                        wSize=wall_size+distMiss;
                        doorN_Pos+=distMiss;
                        Door prev_door=door;
                        door.update(wSize, doorN_Pos);
                        wall_size=wSize;
                        updatePosition(0,prev_door.getN()-door.getN());
                        updatePosition(1,-(prev_door.getWallSize()-prev_door.getP())+(door.getWallSize()-door.getP()));
                    }
                }
            }
        }
        else{
            if(wSize<wallMin){
                float distMiss=(wallMin-wSize);
                float change=min(distMiss, marginNI);
                wSize=wall_size+change;
                doorN_Pos+=change;
                Door prev_door=door;
                door.update(wSize, doorN_Pos);
                wall_size=wSize;
                updatePosition(0,prev_door.getN()-door.getN());
                updatePosition(1,-(prev_door.getWallSize()-prev_door.getP())+(door.getWallSize()-door.getP()));

                distMiss=wallMin-wSize;
                if((marginPI-distMiss)>=wallMin){
                    wSize=wall_size+distMiss;
                    Door prev_door=door;
                    door.updateIWS(wSize);
                    wall_size=wSize;
                    updatePosition(1,-(prev_door.getWallSize()-prev_door.getP())+(door.getWallSize()-door.getP()));
                }
                else{
                    if(marginPI>=distMiss){
                        wSize=wall_size+marginPI;
                        Door prev_door=door;
                        door.updateIWS(wSize);
                        wall_size=wSize;
                        updatePosition(1,-(prev_door.getWallSize()-prev_door.getP())+(door.getWallSize()-door.getP()));
                    }
                    else{
                        return false;
                    }
                }
            }
        }

        //correct_limits(1);
        //door.update(wall_size, abs(chooseDir(pt[0]-pt[2])));

        if(!limits.inside(getInit().x,getInit().y)){
            cout<<"Error from Door Init: ";
            cout<<getInit().x<<" "<<getInit().y<<endl;
        }
        if(!limits.inside(getEnd().x,getEnd().y)){
            cout<<"Error from Door End: ";
            cout<<getEnd().x<<" "<<getEnd().y<<endl;
        }
        if(wall_size<wallMin){
            cout<<"Error Correct wallmin: ";
            cout<<wall_size<<"=size which is < minsize="<<wallMin<<endl;
        }
        if(wall_size>wallMax){
            cout<<"Error Correct wallmax: ";
            cout<<wall_size<<"=size which is > maxsize="<<wallMax<<endl;
        }
        if((wall_size-(abs(getEnd().x-getInit().x)+abs(getEnd().y-getInit().y)))>0.01){
            cout<<"Error Correct wall size diff pts: ";
            cout<<wall_size<<"; "<<getInit().x<<" "<<getInit().y<<"; "<<getEnd().x<<" "<<getEnd().y<<endl;
        }
        if( (chooseDir(pt[2]-pt[0])*sign)<0 || (chooseDir(pt[3]-pt[2])*sign)<0 || (chooseDir(pt[1]-pt[3])*sign)<0 ){
            cout<<"Error Correct pt order: ";
            cout<<getInit().x<<" "<<getInit().y<<"; "<<pt[2].x<<" "<<pt[2].y<<"; "<<pt[3].x<<" "<<pt[3].y<<"; "<<getEnd().x<<" "<<getEnd().y<<endl;
        }
        if( ((wall_size-door.getWallSize())>0.01) || (abs(abs(chooseDir(pt[3]-pt[2]))-door.getSize())>0.01) || (abs(abs(chooseDir(pt[2]-pt[0]))-door.getN())>0.01) || (abs(abs(chooseDir(pt[1]-pt[3]))-abs(door.getWallSize()-door.getP()))>0.01) ){
            cout<<"Error Correct door wall integrity: Wall: ";
            cout<<getInit().x<<" "<<getInit().y<<"; "<<pt[2].x<<" "<<pt[2].y<<"; "<<pt[3].x<<" "<<pt[3].y<<"; "<<getEnd().x<<" "<<getEnd().y<<"; ";
            cout<<"Door: "<<door.getWallSize()<<"; "<<door.getSize()<<" "<<door.getN()<<"; "<<door.getP()<<endl;
        }
        if( (abs(door.getSize()-abs(door.getP()-door.getN()))>0.01) || (door.getSize()>doorMax) || (door.getSize()<doorMin) ){
            cout<<"Error Door integrity: Door limits: ";
            cout<<doorMin<<"; "<<doorMax<<"; ";
            cout<<"Door: "<<door.getWallSize()<<"; "<<door.getSize()<<" "<<door.getN()<<"; "<<door.getP()<<endl;
        }

        return true;
    }
    void createDoor(float dmin, float dmax){
        hasDoor=true;
        door=Door(dmin, dmax, wall_size);
        updatePosition(2, door.getN());
        updatePosition(3, door.getP());
    }
    template<class T>
    void print(T& ss){
        if(!hasDoor)
            ss<<pt[0].x<<" "<<pt[0].y<<" "<<pt[1].x<<" "<<pt[1].y<<endl;
        else{
            ss<<pt[0].x<<" "<<pt[0].y<<" "<<pt[2].x<<" "<<pt[2].y<<endl;
            ss<<pt[3].x<<" "<<pt[3].y<<" "<<pt[1].x<<" "<<pt[1].y<<endl;
        }
    }
    bool has_door(void){
        return hasDoor;
    }
    cv::Point2f getPoint(int index){
        if(index<0)
            index=0;
        else if(index>3)
            index=3;
        return pt[index];
    }
    cv::Point2f getInit(void){
        return pt[0];
    }
    cv::Point2f getEnd(void){
        return pt[1];
    }
    void updateRoomID(unsigned int roomID_){
        roomID=roomID_;
    }
    unsigned int getID(void){
        return id;
    }
    float getSize(void){
        return wall_size;
    }
    Sign getSign(void){
        if(sign>0)
            return Pos;
        else
            return Neg;
    }
    float getFSign(void){
        return sign;
    }
    Dir getDir(void){
        return dir;
    }
    static unsigned int getCountID(void){
        return nID;
    }
    float getConstPos(void){
        return chooseConstDir(pt[0]);
    }
    float getInitChangingPos(void){
        return chooseDir(getInit());
    }
    float getEndChangingPos(void){
        return chooseDir(getEnd());
    }
    float getChangingPos(Sign sign_){
        if( (sign>0 && sign_==Pos) || (sign<0 && sign_==Neg) )
            return chooseDir(pt[1]);
        else
            return chooseDir(pt[0]);
    }
    GD getGD(void) const{
        return gd;
    }
    void setGD(GD gd_){
        gd=gd_;
    }
};

unsigned int Wall::nID=0;

class wallnode{
private:
    unsigned int wall_id;
    int priority;
public:
    wallnode(unsigned int id_, int pr): wall_id(id_), priority(pr){
    }
    wallnode(unsigned int id_): wall_id(id_){
    }
    int getPriority(void) const{
        return priority;
    }
    void updatePriority(int pr){
        priority=pr;
    }
    unsigned int getID(void) const{
        return wall_id;
    }
};

bool operator<(const wallnode & a, const wallnode & b)
{
  return a.getPriority() < b.getPriority();
}

class Room{
private:
    static unsigned int nID;
    unsigned int id;
    cv::Point2f pt;
    vector<Wall> walls;
    vector<GD> gds;
    float wallMin;
    float wallMax;
    float doorMin;
    float doorMax;
    Limits limits;
    unsigned int level;
    void generate(){
        walls.push_back(Wall(pt, Hor, Pos, wallMin, wallMax, doorMin, doorMax, limits, id, GD(Ver,Neg)));
        //cout<<"!!!"<<walls[0].getID()<<endl;
        //cout<<walls[0].getInit().x<<";"<<walls[0].getInit().y<<" to "<<walls[0].getEnd().x<<";"<<walls[0].getEnd().y<<endl;
        walls.push_back(Wall(pt, Ver, Pos, wallMin, wallMax, doorMin, doorMax, limits, id, GD(Hor,Neg)));
        //cout<<"!!!"<<walls[1].getID()<<endl;
        //cout<<walls[1].getInit().x<<";"<<walls[1].getInit().y<<" to "<<walls[1].getEnd().x<<";"<<walls[1].getEnd().y<<endl;
        walls.push_back(walls[0]);
        walls[2].update(walls[1].getEnd());
        walls[2].setGD(GD(Ver,Pos));
        //cout<<"!!!"<<walls[2].getID()<<endl;
        //cout<<walls[2].getInit().x<<";"<<walls[2].getInit().y<<" to "<<walls[2].getEnd().x<<";"<<walls[2].getEnd().y<<endl;
        walls.push_back(walls[1]);
        walls[3].update(walls[0].getEnd());
        walls[2].setGD(GD(Hor,Pos));
        //cout<<"!!!"<<walls[3].getID()<<endl;
        //cout<<walls[3].getInit().x<<";"<<walls[3].getInit().y<<" to "<<walls[3].getEnd().x<<";"<<walls[3].getEnd().y<<endl;
    }
    void generate_from1Wall(Wall prev){
        GD gd=prev.getGD();
        Dir dir=gd.dir;
        Sign sign=gd.sign;
        //cout<<prev.getInit().x<<";"<<prev.getInit().y<<" to "<<prev.getEnd().x<<";"<<prev.getEnd().y<<endl;
        //cout<<prev.getSize()<<endl;
        if(!prev.updateFromDoor())
            return;
        prev.updateRoomID(id);
        walls.push_back(prev);
        walls[0].setGD(GD(dir,neg(sign)));
        //cout<<walls[0].getID()<<endl;
        //cout<<walls[0].getInit().x<<";"<<walls[0].getInit().y<<" to "<<walls[0].getEnd().x<<";"<<walls[0].getEnd().y<<endl;
        //cout<<prev.getSize()<<endl;
        walls.push_back(Wall(walls[0].getInit(), dir, sign, wallMin, wallMax, doorMin, doorMax, limits, id, GD(neg(dir),neg(walls[0].getSign()))));
        //cout<<walls[1].getID()<<endl;
        //cout<<walls[1].getInit().x<<";"<<walls[1].getInit().y<<" to "<<walls[1].getEnd().x<<";"<<walls[1].getEnd().y<<endl;
        walls.push_back(walls[0]);
        walls[2].update(walls[1].getEnd());
        walls[2].setGD(GD(dir,sign));
        //cout<<walls[2].getID()<<endl;
        //cout<<walls[2].getInit().x<<";"<<walls[2].getInit().y<<" to "<<walls[2].getEnd().x<<";"<<walls[2].getEnd().y<<endl;
        //cout<<prev.getSize()<<endl;
        walls.push_back(walls[1]);
        walls[3].update(walls[0].getEnd());
        walls[3].setGD(GD(neg(dir),walls[0].getSign()));
        //cout<<walls[3].getID()<<endl;
        //cout<<walls[3].getInit().x<<";"<<walls[3].getInit().y<<" to "<<walls[3].getEnd().x<<";"<<walls[3].getEnd().y<<endl;
    }
public:
    Room(cv::Point2f p0_, float wmin, float wmax, float dmin, float dmax, Limits limits_):
            pt(p0_), wallMin(wmin), wallMax(wmax), doorMin(dmin), doorMax(dmax), limits(limits_) {
        id=nID++;
        walls.clear();
        gds.clear();
        generate();
        level=0;
    }
    Room(float wmin, float wmax, float dmin, float dmax, Limits limits_, Wall prev):
            wallMin(wmin), wallMax(wmax), doorMin(dmin), doorMax(dmax), limits(limits_) {
        id=nID++;
        walls.clear();
        gds.clear();
        generate_from1Wall(prev);
        level=1;
    }
    Room(){
        walls.clear();
        gds.clear();
    }
    template<class T>
    void print(T& map){
        for(int i=0; i<walls.size();i++){
            walls[i].print(map);
        }
    }
    void getWalls(priority_queue<wallnode>& pq, vector<Wall> & walls_, vector<int> & open_walls_, vector<bool> & closed_walls_){
        for(unsigned int i=level; i<walls.size();i++){
            //cout<<"Adding - "<<walls[i].getID()<<endl;
            if(!walls[i].has_door())
                continue;
            int priority=0;
            pq.push(wallnode(walls[i].getID(),priority));
            if((walls[i].getID()+1)>walls_.size()){
                walls_.resize(walls[i].getID()+1);
                open_walls_.resize(walls[i].getID()+1,0);
                closed_walls_.resize(walls[i].getID()+1,false);
            }
            walls_[walls[i].getID()]=walls[i];
            open_walls_[walls[i].getID()]=priority;
        }
    }
    static unsigned int getCountID(void){
        return nID;
    }
    unsigned int getID(void){
        return id;
    }
    bool empty(void){
        return walls.size()==0;
    }
    Wall getFirstWall(void){
        return walls[0];
    }
};

unsigned int Room::nID=0;

void RoomGen::process(ofstream& map){
    res=0.1;
    height=200;
    width=200;
    map<<res<<" "<<height<<" "<<width<<" "<<endl;
    map<<0<<" "<<0<<" "<<0<<" "<<endl;
    float wallMin=2.0, wallMax=6.0, doorMin=0.8, doorMax=2.0; // wallMax>=2*wallMin; doorMax<=wallMin??...
    // && ??? with wallMx size, it should be impossible for wall to be with less than wallMin distance to border in both sides;????

    Limits limits(5.0*res, ((float)height-5.0)*res, 5.0*res, ((float)width-5.0)*res);

    cv::Point2f p0(5,5);
    //cv::Point2f p0(5.0*res,5.0*res);

    if(!limits.inside(p0.x,p0.y))
        return;

    Room room(p0, wallMin, wallMax, doorMin, doorMax, limits);

    priority_queue<wallnode> pq;

    vector<int> open_walls(0, 0);
    vector<bool> closed_walls(0, false);
    vector<Wall> walls(0);

    vector<Room> rooms(0);
    vector<int> wall2wall(0,-1);

    if(!room.empty()){
        room.print(map);
        room.getWalls(pq, walls, open_walls, closed_walls);
        if((room.getID()+1)>rooms.size()){
            rooms.resize(room.getID()+1);
        }
        rooms[room.getID()]=room;
    }

    while(!pq.empty())
    {
        Wall wall=walls[pq.top().getID()];
        pq.pop();
        //cout<<wall.getID()<<endl;
        if(closed_walls[wall.getID()])
            continue;
        closed_walls[wall.getID()]=true;

        Room room_(wallMin, wallMax, doorMin, doorMax, limits, wall);
        if(!room.empty()){
            room_.print(map);
            room_.getWalls(pq, walls, open_walls, closed_walls);
            if((room.getID()+1)>rooms.size()){
                rooms.resize(room.getID()+1);
            }
            rooms[room.getID()]=room;

            unsigned int wallMaxID=max(room_.getFirstWall().getID(),wall.getID());
            if((wallMaxID+1)>wall2wall.size()){
                wall2wall.resize(wallMaxID+1,-1);
            }
            wall2wall[wall.getID()]=room_.getFirstWall().getID();
            wall2wall[room_.getFirstWall().getID()]=wall.getID();
            //test if initial and created wall have the same opening
        }
        else{
            //// TODO: if not possible to create room, close wall!
        }
        if(Wall::getCountID()>100)
           break;
    }
    cout<<"Number walls: "<<Wall::getCountID()<<endl;
    cout<<"Number rooms: "<<Room::getCountID()<<endl;
}

bool RoomGen::run(void){
    ofstream myfile;
    string mapFile = ros::package::getPath("map_transform").append("/maps/map.txt");
    myfile.open(mapFile);
    if (myfile.is_open()){
        process(myfile);
        return true;
    }
    else{
        //ROS_INFO("Failed to open Map file.");
    }
    myfile.close();
    return false;
}

bool RoomGen::finish(void){
    std_srvs::Empty srv;
    return permission.call(srv);
}

bool suc=false;

// Replacement SIGINT handler
void HandlerStop(int){
    if(!suc)
        ROS_INFO("Failed to generate Map file.");
    ros::shutdown();
    exit(-1);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "rooms", ros::init_options::NoSigintHandler);
  signal(SIGINT, HandlerStop);
  ros::NodeHandle nh;//("~");
  RoomGen rooms(nh);
  ros::Rate loop_rate(10);
  while (ros::ok()){
    ros::spinOnce();
    if(!suc){
        if(rooms.run()){
            suc=true;
            ROS_INFO("Successfully generated Map file.");
        }
    }
    else{
        if (rooms.finish()){
            ROS_INFO("Successful request for Map Generation");
            break;
        }
        else{
            ROS_INFO("Failed to request Map Generation");
        }
    }
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}
