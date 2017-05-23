#include "ros/ros.h"
#include <vector>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_listener.h"
#include "std_srvs/Empty.h"

#include "Astar.hpp"

using namespace std;

class Multirobotplannersensing{
private:
    ros::NodeHandle nh_;

    ros::Publisher pub1;
    ros::Publisher pub2;

    ros::Publisher pub_markers;
    
    ros::Subscriber sub1;

    ros::Subscriber sub2;

    ros::ServiceServer service;

    ros::ServiceServer service2;

    ros::Subscriber sub_goals;

    tf::TransformListener pos_listener;

    void rcv_map1(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    void rcv_map2(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    bool ask_plan(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

    bool clear(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

    void rcv_goal(const geometry_msgs::PoseStamped::ConstPtr& msg);

    vector<vector<vector<bool> > >  msg_rcv;

    vector<int> count;

    vector<bool> map_rcv;

    bool pl;

    bool getMapValue(int n, int i, int j);

    vector<geometry_msgs::Point> goals;

    int prev_goals;

    float res;

    int width,height;

    PointI convertW2I(geometry_msgs::Point p);

    geometry_msgs::Point convertI2W(PointI p);

    //double distance2Path(vector<PointI> path, PointI goal, int r);

    //int minDistanceGoal2Path(vector<PointI> path, vector<PointI> goal, int r);

public:

    Multirobotplannersensing(ros::NodeHandle nh): nh_(nh)
    {

        pub1 = nh_.advertise<nav_msgs::Path>("path0", 1,true);
        pub2 = nh_.advertise<nav_msgs::Path>("path1", 1,true);

        pub_markers = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1,true);

        sub1 = nh_.subscribe("robot_0/v_map", 1, &Multirobotplannersensing::rcv_map1, this);

        sub2 = nh_.subscribe("robot_1/v_map", 1, &Multirobotplannersensing::rcv_map2, this);


        service = nh_.advertiseService("plan", &Multirobotplannersensing::ask_plan, this);

        count.assign(2,0);

        map_rcv.assign(2,false);

        msg_rcv.resize(2);

        pl=false;

        goals.clear();

        prev_goals=goals.size();

    }

    ~Multirobotplannersensing()
    {

    }

    void plan(void);

    void publish(void);
};


void Multirobotplannersensing::rcv_map1(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    msg_rcv[0].assign(msg->info.width, vector<bool>(msg->info.height,false));

    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
                if(*mapDataIterC == 0)
                {
                    msg_rcv[0][j][i]=true;
                }
                else
                {
                    msg_rcv[0][j][i]=false;
                }

                mapDataIterC++;
            }
    }

    map_rcv[0]=true;

    count[0]=count[0]+1;

    res=msg->info.resolution;
    width=msg->info.width;
    height=msg->info.height;

}

void Multirobotplannersensing::rcv_map2(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    msg_rcv[1].assign(msg->info.width, vector<bool>(msg->info.height,false));

    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
                if(*mapDataIterC == 0)
                {
                    msg_rcv[1][j][i]=true;
                }
                else
                {
                    msg_rcv[1][j][i]=false;
                }

                mapDataIterC++;
            }
    }

    map_rcv[1]=true;

    count[1]=count[1]+1;
}


bool Multirobotplannersensing::getMapValue(int n, int i, int j)
{
    return msg_rcv[n][i][j];
}

bool Multirobotplannersensing::ask_plan(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
    pl=true;
    return true;
}


bool Multirobotplannersensing::clear(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
    pl=false;
    goals.clear();

    nav_msgs::Path path_0,path_1;
    path_0.poses.clear();
    path_0.header.frame_id = "/map";
    path_0.header.stamp =  ros::Time::now();
    path_0.poses.clear();
    path_1=path_0;

    pub1.publish(path_0);

    pub2.publish(path_1);

    return true;
}

void Multirobotplannersensing::rcv_goal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goals.push_back(msg->pose.position);

}

PointI Multirobotplannersensing::convertW2I(geometry_msgs::Point p)
{
    PointI pf(p.x/res,p.y/res);

    return pf;
}

geometry_msgs::Point Multirobotplannersensing::convertI2W(PointI p)
{
    geometry_msgs::Point pf;
    pf.x=p.i*res;
    pf.y=p.j*res;

    return pf;
}


//double Multirobotplannersensing::distance2Path(vector<PointI> path, PointI goal, int r)
//{
//    vector<double> heuristic(0);
//    vector<int> pos(0);

//    for(unsigned int p=0;p<path.size();p++)
//    {
//        if (!msg_rcv[r][path[p].i][path[p].j])
//            continue;

//        double heur=(double) (path[p].i-goal.i)*(path[p].i-goal.i)+(path[p].j-goal.j)*(path[p].j-goal.j);

//        vector<double>::iterator heurIt = heuristic.begin();
//        vector<int>::iterator posIt = pos.begin();

//        while(heurIt!=heuristic.end())
//        {
//            if(heur<*heurIt)
//                break;
//            heurIt++;
//            posIt++;
//        }

//        heuristic.insert(heurIt,heur);
//        pos.insert(posIt,p);
//    }

//    double min_real=0;
//    Apath pathT;

//    for(unsigned int h=0;h<(heuristic.size());h++)
//    {
//        pathT=Astar(path[pos[h]], goal,r);
//        if(h==0)
//            min_real=pathT.cost;
//        else
//        {
//            if(pathT.cost<min_real)
//                min_real=pathT.cost;

//            if(h<(heuristic.size()-1))
//            {
//                if( (min_real*min_real)<heuristic[h+1])
//                    return min_real;
//                else
//                    continue;
//            }
//        }
//    }

//    return pathT.cost;
//}

//int Multirobotplannersensing::minDistanceGoal2Path(vector<PointI> path, vector<PointI> goal, int r)
//{
//    vector<double> heuristic(0);
//    vector<int> pos(0);
//    vector<int> gp(0);

//    for(unsigned int g=0;g<goal.size();g++)
//    {
//        for( unsigned int p=0;p<path.size();p++)
//        {
//            if (!msg_rcv[r][path[p].i][path[p].j])
//                continue;

//            double heur=(double) (path[p].i-goal[g].i)*(path[p].i-goal[g].i)+(path[p].j-goal[g].j)*(path[p].j-goal[g].j);

//            vector<double>::iterator heurIt = heuristic.begin();
//            vector<int>::iterator posIt = pos.begin();
//            vector<int>::iterator gpIt = gp.begin();

//            while(heurIt!=heuristic.end())
//            {
//                if(heur<*heurIt)
//                    break;
//                heurIt++;
//                posIt++;
//                gpIt++;
//            }

//            heuristic.insert(heurIt,heur);
//            pos.insert(posIt,p);
//            gp.insert(gpIt,g);
//        }
//    }

//    double min_real=0;
//    int g_real=0;
//    Apath pathT;

//    for(unsigned int h=0;h<(heuristic.size());h++)
//    {
//        pathT=Astar(path[pos[h]], goal[gp[h]],r);
//        if(h==0)
//        {
//            min_real=pathT.cost;
//            g_real=gp[h];
//        }
//        else
//        {
//            if(pathT.cost<min_real)
//            {
//                min_real=pathT.cost;
//                g_real=gp[h];
//            }

//            if(h<(heuristic.size()-1))
//            {
//                if( (min_real*min_real)<heuristic[h+1])
//                    return g_real;
//                else
//                    continue;
//            }
//        }
//    }

//    return g_real;
//}


class Sequence {
public:
    vector<vector<int> > seq;
    vector<vector<int> > rem;

    Sequence()
    {
        seq.clear();
        rem.clear();
    }

    void append(Sequence b)
    {
        seq.insert(seq.end(),b.seq.begin(),b.seq.end());
        rem.insert(rem.end(),b.rem.begin(),b.rem.end());
    }

    void print(void)
    {
        for (unsigned int i=0;i<seq.size();i++)
        {
            cout<<"Permutation: ";
            for (unsigned int j=0;j<seq[i].size();j++)
            {
                cout<<seq[i][j]<<" ";
            }
            cout<<"Remainder: ";
            for (unsigned int j=0;j<rem[i].size();j++)
            {
                cout<<rem[i][j]<<" ";
            }
            cout<<endl;
        }
    }
};

Sequence permutations(vector<int> in, int size)
{
    Sequence perm, temp;
    vector<int> poss;

    if(size==0)
    {
        perm.seq.push_back(vector<int>(0));
        perm.rem.push_back(in);
        return perm;
    }

    for (unsigned int i=0;i<in.size();i++)
    {
        poss=in;
        poss.erase(poss.begin()+i);
        temp=permutations(poss,size-1);
        for(unsigned int j=0;j<temp.seq.size();j++)
            temp.seq[j].insert ( temp.seq[j].begin() , in[i]);
        perm.append(temp);
    }

    return perm;
}

Sequence combinations(vector<int> in, unsigned int size)
{
    Sequence comb, temp;
    vector<int> poss, poss_r;

    if(size==0)
    {
        comb.seq.push_back(vector<int>(0));
        comb.rem.push_back(in);
        return comb;
    }

    for (unsigned int i=0;i<in.size();i++)
    {
        poss=in;
        poss.erase(poss.begin(),poss.begin()+i+1);
        poss_r=vector<int>(in.begin(),in.begin()+i);
        temp=combinations(poss,size-1);
        for(unsigned int j=0;j<temp.seq.size();j++)
        {
            temp.seq[j].insert ( temp.seq[j].begin() , in[i]);
            temp.rem[j].insert ( temp.rem[j].begin() , poss_r.begin(), poss_r.end());
        }
        comb.append(temp);
    }

    return comb;
}

Sequence combine(vector<int> g1, vector<int> g2, vector<int> gc)
{
    Sequence ret, comb, temp, temp2;

    for(unsigned int i=0;i<=gc.size();i++)
    {
        temp=combinations(gc,i);
        comb.append(temp);
    }

    vector<int> temp_s, temp_s2;

    for(unsigned int i=0;i<comb.seq.size();i++)
    {
        temp_s=g1;
        temp_s.insert(temp_s.end(),comb.seq[i].begin(),comb.seq[i].end());

        temp_s2=g2;
        temp_s2.insert(temp_s2.end(),comb.rem[i].begin(),comb.rem[i].end());


        temp=permutations(temp_s,temp_s.size());

        for(unsigned int j=0;j<temp.seq.size();j++)
        {

            temp2=permutations(temp_s2,temp_s2.size());

            for(unsigned int k=0; k<temp2.seq.size();k++)
            {
                ret.seq.push_back(temp.seq[j]);
                ret.rem.push_back(temp2.seq[k]);
            }
        }
    }

    return ret;
}

double cost_function(double a, double b)
{
    return max(a,b);
}

void Multirobotplannersensing::plan(void)
{
    if(map_rcv[0] && map_rcv[1] && pl && (goals.size()>0) )
    {

        nav_msgs::Path path_0,path_1;

        vector<vector<PointI> > pr(2,vector<PointI>(0));

        vector< vector<int> > pg(2,vector<int>(0)), gr(4,vector<int>(0));

        vector<PointI> g;

        vector<double> pc(2,0.0);


        path_0.poses.clear();
        path_0.header.frame_id = "/map";
        path_0.header.stamp =  ros::Time::now();
        path_1=path_0;


        geometry_msgs::PoseStamped p_temp;
        p_temp.header.frame_id   = "/map";
        p_temp.header.stamp =  ros::Time::now();

        p_temp.pose.orientation.w=1;

        tf::StampedTransform transform;
        try{
            pos_listener.lookupTransform("/map", "/robot_0/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_INFO("%s",ex.what());
          return ;
        }

        PointI pi_temp( (int) round((transform.getOrigin().x())/res), (int) round((transform.getOrigin().y())/res) );

        p_temp.pose.position=convertI2W(pi_temp);

        path_0.poses.push_back(p_temp);

        pr[0].push_back(pi_temp);

        g.push_back(pr[0].back());


        try{
            pos_listener.lookupTransform("/map", "/robot_1/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_INFO("%s",ex.what());
          return ;
        }

        pi_temp=PointI( (int) round((transform.getOrigin().x())/res), (int) round((transform.getOrigin().y())/res) );

        p_temp.pose.position=convertI2W(pi_temp);

        path_1.poses.push_back(p_temp);

        pr[1].push_back(pi_temp);

        g.push_back(pr[1].back());


        for(unsigned int i=0;i<goals.size();i++)
        {
            p_temp.pose.position=goals[i];

            pi_temp=convertW2I(p_temp.pose.position);

            g.push_back(pi_temp);

            if( msg_rcv[0][pi_temp.i][pi_temp.j] && !msg_rcv[1][pi_temp.i][pi_temp.j] )
                gr[0].push_back(2+i);
            else if ( msg_rcv[1][pi_temp.i][pi_temp.j] && !msg_rcv[0][pi_temp.i][pi_temp.j] )
                gr[1].push_back(2+i);
            else if ( msg_rcv[1][pi_temp.i][pi_temp.j] && msg_rcv[0][pi_temp.i][pi_temp.j] )
                gr[2].push_back(2+i);
            else
                gr[3].push_back(2+i);

        }

        vector<bool> gt(g.size(),false);

        int g0s=gr[0].size(), g1s=gr[1].size(),gbs=gr[2].size();

        vector<vector<vector<Apath> > > mat(g.size(),vector<vector<Apath> >(g.size(),vector<Apath>(2,Apath())));


        Apath path;

        pg[0].push_back(0);
        pg[1].push_back(1);


        /////////////////////////// METHOD 1 ///////////////////////////////////////

        ros::Time t01=ros::Time::now();

        cout<<"----> Method 1:"<<endl;



    }
}



void Multirobotplannersensing::publish(void)
{
    if(goals.size()>0)
    {
        visualization_msgs::MarkerArray points;
        visualization_msgs::Marker point;
        point.header.frame_id = "/map";
        point.header.stamp =  ros::Time::now();
        point.ns = "points_and_lines";
        point.action = visualization_msgs::Marker::ADD;
        point.pose.orientation.w = 1.0;

        point.type = visualization_msgs::Marker::SPHERE;

        point.scale.x = 0.2;
        point.scale.y = 0.2;
        point.scale.z = 0.2;

        point.color.g = 1.0f;
        point.color.a = 1.0;

        point.lifetime = ros::Duration(2);

        for (uint32_t i = 0; i < goals.size(); ++i)
        {
          point.pose.position=goals[i];
          point.id = i;

          points.markers.push_back(point);
        }

        pub_markers.publish(points);

        prev_goals=goals.size();
    }
}


int main(int argc, char **argv)
{

  
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh("~");


  Multirobotplannersensing planner(nh);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {


    ros::spinOnce();

    planner.plan();

    planner.publish();

    loop_rate.sleep();
  }


  return 0;
}
