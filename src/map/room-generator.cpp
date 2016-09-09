#include "ros/ros.h"
#include "ros/package.h"
#include <signal.h>
#include <fstream>
#include <opencv2/core/core.hpp>
#include "std_srvs/Empty.h"
#include <queue>

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
        if(doorN<0)
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

class Wall{
private:
    static int nID;
    int id;
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
    void updatePosition(int index, float size){
        if(dir==Hor)
            pt[index].y+=sign*size;
        else
            pt[index].x+=sign*size;
    }
    void generate(){
        wall_size=wallMin+((float)rand()/RAND_MAX)*(wallMax-wallMin);
        updatePosition(1, wall_size);
    }
public:
    Wall(cv::Point2f p0_, Dir dir_, Sign sign_, float wmin, float wmax, float dmin, float dmax):
            id(nID++), dir(dir_), wallMin(wmin), wallMax(wmax), doorMin(dmin), doorMax(dmax) {
        if(sign_==Pos)
            sign=1.0;
        else
            sign=-1.0;
        pt.assign(4,p0_);
        hasDoor=false;
        generate();
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
        createDoor(doorMin, doorMax);
    }
    bool updateFromDoor(void){
        if(!hasDoor)
            return false;
        id=nID++;
        wall_size=max(wallMin,door.getSize())+((float)rand()/RAND_MAX)*(wallMax-max(wallMin,door.getSize()));
        door.update(wall_size);
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
    int getID(void){
        return id;
    }
};

int Wall::nID=0;

class wallnode{
private:
    int wall_id;
    int priority;
public:
    wallnode(int id_, int pr): wall_id(id_), priority(pr){
    }
    wallnode(int id_): wall_id(id_){
    }
    int getPriority(void) const{
        return priority;
    }
    void updatePriority(int pr){
        priority=pr;
    }
    int getID(void) const{
        return wall_id;
    }
};

bool operator<(const wallnode & a, const wallnode & b)
{
  return a.getPriority() < b.getPriority();
}

class Room{
private:
    cv::Point2f pt;
    vector<Wall> walls;
    float wallMin;
    float wallMax;
    float doorMin;
    float doorMax;
    void generate(){
        walls.push_back(Wall(pt, Hor, Pos, wallMin, wallMax, doorMin, doorMax));
        walls.push_back(Wall(pt, Ver, Pos, wallMin, wallMax, doorMin, doorMax));
        walls.push_back(walls[0]);
        walls[2].update(walls[1].getEnd());
        walls.push_back(walls[1]);
        walls[3].update(walls[0].getEnd());
    }
public:
    Room(cv::Point2f p0_, float wmin, float wmax, float dmin, float dmax):
            pt(p0_), wallMin(wmin), wallMax(wmax), doorMin(dmin), doorMax(dmax) {
        walls.clear();
        generate();
    }
    template<class T>
    void print(T& map){
        for(int i=0; i<walls.size();i++){
            walls[i].print(map);
        }
    }
};

void RoomGen::process(ofstream& map){
    res=0.1;
    height=200;
    width=200;
    map<<res<<" "<<height<<" "<<width<<" "<<endl;
    map<<0<<" "<<0<<" "<<0<<" "<<endl;
    float wallMin=2.0, wallMax=6.0, doorMin=0.8, doorMax=2.0;
    Room room(cv::Point2f(5,5), wallMin, wallMax, doorMin, doorMax);
    room.print(map);

    priority_queue<wallnode> pq;
    Wall test(cv::Point2f(0.0), Hor, Pos, 2, 5, 0.5, 1.5);
    pq.push(wallnode(test.getID(),2));

    vector<int> open_walls(pq.top().getID()+1, 0);
    vector<bool> closed_walls(pq.top().getID()+1, false);
    vector<Wall> walls(pq.top().getID()+1);
    walls[pq.top().getID()]=test;
    open_walls[pq.top().getID()]=pq.top().getPriority();

    while(!pq.empty())
    {
       Wall wall=walls[pq.top().getID()];
       pq.pop();
       //cout<<wall.getID()<<endl;
       if(closed_walls[wall.getID()])
           continue;
       closed_walls[wall.getID()]=true;
       //extend pq with new walls
    }
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
