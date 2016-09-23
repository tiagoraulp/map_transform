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
public:
    Door(float dmin, float dmax, float wall_): doorMin(dmin), doorMax(dmax), wall(wall_){
        size=doorMin+((float)rand()/RAND_MAX)*(doorMax-doorMin);
        generate();
    }
    Door(){}
    float getSize(){
        return size;
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
            return xmax-x;

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
    void left(cv::Point2f& pt){
        left(pt.x, pt.y);
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
};

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
    void correct_limits(int ind){
        if(!limits.inside(pt[ind].x,pt[ind].y)){
            wall_size-=limits.diff(pt[ind].x,pt[ind].y)<0?0:limits.diff(pt[ind].x,pt[ind].y);
            limits.fix(pt[ind].x,pt[ind].y);
        }
        else{
            float xleft=pt[ind].x, yleft=pt[ind].y;
            limits.left(xleft, yleft);
            float diff=chooseDir(xleft,yleft);
            if(diff>=wallMin)
                return;
            else{
                float sign_;
                if(ind==1)
                    sign_=1;
                else
                    sign_=-1;
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
        }
    }
    void generate(){
        wall_size=wallMin+((float)rand()/RAND_MAX)*(wallMax-wallMin);
        updatePosition(1, wall_size);
        correct_limits(1);
    }
public:
    Wall(cv::Point2f p0_, Dir dir_, Sign sign_, float wmin, float wmax, float dmin, float dmax, Limits limits_):
            id(nID++), dir(dir_), wallMin(wmin), wallMax(wmax), doorMin(dmin), doorMax(dmax), limits(limits_) {
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
        correct_limits(0);
        correct_limits(1);
        return true;
    }
    void createDoor(float dmin, float dmax){
        hasDoor=true;
        door=Door(dmin, dmax, wall_size);
        //door.generate();
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
//    cv::Point2f getPoint(int index){
//        if(index<0)
//            index=0;
//        else if(index>3)
//            index=3;
//        return pt[index];
//    }
    cv::Point2f getInit(void){
        return pt[0];
    }
    cv::Point2f getEnd(void){
        return pt[1];
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
};

unsigned int Wall::nID=0;

struct growing_direction{
    Dir dir;
    Sign sign;
    growing_direction(Dir dir_, Sign sign_): dir(dir_), sign(sign_){
    }
};

typedef growing_direction GD;

class wallnode{
private:
    unsigned int wall_id;
    GD gd;
    int priority;
public:
    wallnode(unsigned int id_, GD gd_, int pr): wall_id(id_), gd(gd_), priority(pr){
    }
    wallnode(unsigned int id_, GD gd_): wall_id(id_), gd(gd_){
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
    GD getGD(void) const{
        return gd;
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
        walls.push_back(Wall(pt, Hor, Pos, wallMin, wallMax, doorMin, doorMax, limits));
        //cout<<"!!!"<<walls[0].getID()<<endl;
        //cout<<walls[0].getInit().x<<";"<<walls[0].getInit().y<<" to "<<walls[0].getEnd().x<<";"<<walls[0].getEnd().y<<endl;
        walls.push_back(Wall(pt, Ver, Pos, wallMin, wallMax, doorMin, doorMax, limits));
        //cout<<"!!!"<<walls[1].getID()<<endl;
        //cout<<walls[1].getInit().x<<";"<<walls[1].getInit().y<<" to "<<walls[1].getEnd().x<<";"<<walls[1].getEnd().y<<endl;
        walls.push_back(walls[0]);
        walls[2].update(walls[1].getEnd());
        //cout<<"!!!"<<walls[2].getID()<<endl;
        //cout<<walls[2].getInit().x<<";"<<walls[2].getInit().y<<" to "<<walls[2].getEnd().x<<";"<<walls[2].getEnd().y<<endl;
        walls.push_back(walls[1]);
        walls[3].update(walls[0].getEnd());
        //cout<<"!!!"<<walls[3].getID()<<endl;
        //cout<<walls[3].getInit().x<<";"<<walls[3].getInit().y<<" to "<<walls[3].getEnd().x<<";"<<walls[3].getEnd().y<<endl;
        gds.push_back(GD(Ver,Neg));
        gds.push_back(GD(Hor,Neg));
        gds.push_back(GD(Ver,Pos));
        gds.push_back(GD(Hor,Pos));
    }
    void generate_from1Wall(Wall prev, GD gd){
        Dir dir=gd.dir;
        Sign sign=gd.sign;
        //cout<<prev.getInit().x<<";"<<prev.getInit().y<<" to "<<prev.getEnd().x<<";"<<prev.getEnd().y<<endl;
        //cout<<prev.getSize()<<endl;
        if(!prev.updateFromDoor())
            return;
        walls.push_back(prev);
        //cout<<walls[0].getID()<<endl;
        //cout<<walls[0].getInit().x<<";"<<walls[0].getInit().y<<" to "<<walls[0].getEnd().x<<";"<<walls[0].getEnd().y<<endl;
        //cout<<prev.getSize()<<endl;
        walls.push_back(Wall(walls[0].getInit(), dir, sign, wallMin, wallMax, doorMin, doorMax, limits));
        //cout<<walls[1].getID()<<endl;
        //cout<<walls[1].getInit().x<<";"<<walls[1].getInit().y<<" to "<<walls[1].getEnd().x<<";"<<walls[1].getEnd().y<<endl;
        walls.push_back(walls[0]);
        walls[2].update(walls[1].getEnd());
        //cout<<walls[2].getID()<<endl;
        //cout<<walls[2].getInit().x<<";"<<walls[2].getInit().y<<" to "<<walls[2].getEnd().x<<";"<<walls[2].getEnd().y<<endl;
        //cout<<prev.getSize()<<endl;
        walls.push_back(walls[1]);
        walls[3].update(walls[0].getEnd());
        //cout<<walls[3].getID()<<endl;
        //cout<<walls[3].getInit().x<<";"<<walls[3].getInit().y<<" to "<<walls[3].getEnd().x<<";"<<walls[3].getEnd().y<<endl;
        gds.push_back(GD(dir,neg(sign)));
        gds.push_back(GD(neg(dir),neg(walls[0].getSign())));
        gds.push_back(GD(dir,sign));
        gds.push_back(GD(neg(dir),walls[0].getSign()));
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
    Room(float wmin, float wmax, float dmin, float dmax, Limits limits_, Wall prev, GD gd):
            wallMin(wmin), wallMax(wmax), doorMin(dmin), doorMax(dmax), limits(limits_) {
        id=nID++;
        walls.clear();
        gds.clear();
        generate_from1Wall(prev, gd);
        level=1;
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
            pq.push(wallnode(walls[i].getID(),gds[i],priority));
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
    bool empty(void){
        return walls.size()==0;
    }
};

unsigned int Room::nID=0;

void RoomGen::process(ofstream& map){
    res=0.1;
    height=200;
    width=200;
    map<<res<<" "<<height<<" "<<width<<" "<<endl;
    map<<0<<" "<<0<<" "<<0<<" "<<endl;
    float wallMin=2.0, wallMax=6.0, doorMin=0.8, doorMax=2.0; // wallMax>=2*wallMin;

    Limits limits(5.0*res, ((float)height-5.0)*res, 5.0*res, ((float)width-5.0)*res);

    //cv::Point2f p0(5,5);
    cv::Point2f p0(5.0*res,5.0*res);

    if(!limits.inside(p0.x,p0.y))
        return;

    Room room(p0, wallMin, wallMax, doorMin, doorMax, limits);
    room.print(map);

    priority_queue<wallnode> pq;
    //Wall test(cv::Point2f(0.0), Hor, Pos, 2, 5, 0.5, 1.5);
    //pq.push(wallnode(test.getID(),2));

    //vector<int> open_walls(pq.top().getID()+1, 0);
    //vector<bool> closed_walls(pq.top().getID()+1, false);
    //vector<Wall> walls(pq.top().getID()+1);
    vector<int> open_walls(0, 0);
    vector<bool> closed_walls(0, false);
    vector<Wall> walls(0);

    if(!room.empty())
        room.getWalls(pq, walls, open_walls, closed_walls);

    //walls[pq.top().getID()]=test;
    //open_walls[pq.top().getID()]=pq.top().getPriority();

    while(!pq.empty())
    {
        Wall wall=walls[pq.top().getID()];
        GD gd=pq.top().getGD();
        pq.pop();
        //cout<<wall.getID()<<endl;
        if(closed_walls[wall.getID()])
            continue;
        closed_walls[wall.getID()]=true;

        Room room_(wallMin, wallMax, doorMin, doorMax, limits, wall, gd);
        if(!room.empty()){
            room_.print(map);
            room_.getWalls(pq, walls, open_walls, closed_walls);
            //extend pq with new walls
        }
        if(Wall::getCountID()>60000)
           break;
    }
    cout<<"Number walls: "<<Wall::getCountID()<<endl;
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
