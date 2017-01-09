#include "visC_transf.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "labelling.hpp"
#include "CritPoints.hpp"
#include "ray.hpp"
#include "color.hpp"
#include "brute_force.hpp"

#include "vector_utils.hpp"

#include <cmath>

#include "std_msgs/Int16MultiArray.h"

#include <ros/package.h>

using namespace std;

static const double PI = 3.141592653589793;

static const std::string M_WINDOW = "Map";
static const std::string E_WINDOW = "Erosion";
static const std::string C_WINDOW = "Close";
static const std::string L_WINDOW = "Labelled";
static const std::string A_WINDOW = "Actuation";
static const std::string V_WINDOW = "Visibility";
static const std::string D_WINDOW = "Debug";
static const std::string G_WINDOW = "Ground_Truth";
static const std::string CP_WINDOW = "Comparison";

void VisC_transf::update_config(map_transform::ParametersConfig config, bool ch, bool _opt)
{
    if(_opt || (!_opt && opt && !ch) || ch)
    {
        changed=true;
        changed2=true;

        infl=config.infl;
        defl=config.defl;
        _debug=config.debug;
        gt=config.ground_truth;
        rxr=config.x;
        ryr=config.y;
    }

    if(count>0)
        opt=_opt;
}


VisC_transf::VisC_transf(ros::NodeHandle nh): Vis_transf(nh)
{
    nh_.param("infl", infl, 5);
    nh_.param("defl", defl, infl);
    graph_publisher = nh_.advertise<map_transform::VisCom>("graph", 10,true);
    act_dist_pub = nh_.advertise<std_msgs::Int16MultiArray>("act_dist", 10,true);
    opt=true;
}

void VisC_transf::publish(void)
{
    Vis_transf::publish();
    map_transform::VisCom graphMsg;
    //graphMsg.vis=vis_;
    graphMsg=vis_;

    graphMsg.header.stamp = ros::Time::now();
    graphMsg.header.frame_id = "map";

    graph_publisher.publish(graphMsg);

    if(count>0 && pos_rcv){
        std_msgs::Int16MultiArray act_msg;
        act_msg.layout.data_offset=0;
        act_msg.layout.dim.clear();
        std_msgs::MultiArrayDimension dim;
        dim.label="x";
        dim.size=act_dist.rows;
        dim.stride=act_dist.rows*act_dist.cols;
        act_msg.layout.dim.push_back(dim);
        dim.label="y";
        dim.size=act_dist.cols;
        dim.stride=act_dist.cols;
        act_msg.layout.dim.push_back(dim);
        act_msg.data.resize(act_msg.layout.data_offset+act_msg.layout.dim[0].stride);
        for(int i=0;i<act_dist.rows;i++){
            for(int j=0;j<act_dist.cols;j++){
                act_msg.data[act_msg.layout.data_offset+act_msg.layout.dim[1].stride*i+j]=act_dist(i,j);
            }
        }
        act_dist_pub.publish(act_msg);
    }
}


VisC_transf::~VisC_transf()
{
    if(_debug){
       cv::destroyWindow(M_WINDOW);
       cv::destroyWindow(E_WINDOW);
       cv::destroyWindow(C_WINDOW);
       cv::destroyWindow(L_WINDOW);
       cv::destroyWindow(A_WINDOW);
       cv::destroyWindow(D_WINDOW);
       cv::destroyWindow(V_WINDOW);
       cv::destroyWindow(G_WINDOW);
       cv::destroyWindow(CP_WINDOW);

    }
}


void VisC_transf::show(void)
{
    if(count>0 && _debug){
        cv::imshow(M_WINDOW,map_or);

        string filename = ros::package::getPath("map_transform").append("/images/map.png");
        cv::imwrite(filename, map_or);

        cv::imshow(C_WINDOW,map_closeOp);

        filename = ros::package::getPath("map_transform").append("/images/map_closed.png");
        cv::imwrite(filename, map_closeOp);

        if(pos_rcv)
        {
            cv::imshow(E_WINDOW,map_erosionOpPrintColor);

            filename = ros::package::getPath("map_transform").append("/images/map_erosion_pt.png");
            cv::imwrite(filename, map_erosionOpPrintColor);

            filename = ros::package::getPath("map_transform").append("/images/map_erosion.png");
            cv::imwrite(filename, map_erosionOp);

            cv::imshow(L_WINDOW,map_label);

            filename = ros::package::getPath("map_transform").append("/images/navigable.png");
            cv::imwrite(filename, map_label);

            cv::imshow(A_WINDOW,map_act);

            filename = ros::package::getPath("map_transform").append("/images/actuation.png");
            cv::imwrite(filename, map_act);

            cv::imshow(V_WINDOW,map_vis);

            filename = ros::package::getPath("map_transform").append("/images/visibility.png");
            cv::imwrite(filename, map_vis);

            cv::imshow(D_WINDOW,map_debug);

            filename = ros::package::getPath("map_transform").append("/images/debug.png");
            cv::imwrite(filename, map_debug);

            if(gt)
            {
                cv::imshow(G_WINDOW,map_truth);
                cv::waitKey(2);

                filename = ros::package::getPath("map_transform").append("/images/truth.png");
                cv::imwrite(filename, map_truth);

                cv::imshow(CP_WINDOW,map_comp);
                cv::waitKey(2);

                filename = ros::package::getPath("map_transform").append("/images/comp.png");
                cv::imwrite(filename, map_comp);
            }
            else
            {
                cv::destroyWindow(G_WINDOW);
                cv::waitKey(2);
                cv::destroyWindow(CP_WINDOW);
                cv::waitKey(2);
            }
        }
        else
        {
            cv::imshow(E_WINDOW,map_erosionOp);

            cv::destroyWindow(D_WINDOW);
            cv::waitKey(2);
            cv::destroyWindow(L_WINDOW);
            cv::waitKey(2);
            cv::destroyWindow(A_WINDOW);
            cv::waitKey(2);
            cv::destroyWindow(V_WINDOW);
            cv::waitKey(2);
            cv::destroyWindow(G_WINDOW);
            cv::waitKey(2);
            cv::destroyWindow(CP_WINDOW);
            cv::waitKey(2);
        }
        cv::waitKey(3);
    }

    if(!_debug)
    {
       cv::waitKey(2);
       cv::destroyWindow(M_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(E_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(C_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(L_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(A_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(D_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(V_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(G_WINDOW);
       cv::waitKey(2);
       cv::destroyWindow(CP_WINDOW);
       cv::waitKey(2);
    }
}

bool VisC_transf::conf_space(void)
{
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                           cv::Size( 2*infl + 1, 2*infl+1 ),
                                           cv::Point( infl, infl ) );

    cv::Mat element_d = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                           cv::Size( 2*infl + 1, 2*infl+1 ),
                                           cv::Point( infl, infl ) );

    cv::Mat or_map, er_map, cr_map;

    or_map=cv_map.clone();
    msg_rcv_pub=msg_rcv;

    cv::erode( or_map, er_map, element);//,cv::Point(-1,-1),1,cv::BORDER_CONSTANT,0);//cv::morphologyDefaultBorderValue());
    cv::dilate( er_map, cr_map, element_d);

    map_or=or_map;
    map_erosionOp=er_map;
    map_closeOp=cr_map;

    return true;
}


bool VisC_transf::reachability_map(cv::Point3i pos, cv::Mat & r_map)
{
    std::vector<std::vector<cv::Point> > labels=label(map_erosionOp.clone()/255,8);

    bool found_pos=false, found_prev=false;
    unsigned int label_pos=0, prev_label=0;

    for (unsigned int i=0;i<labels.size();i++){
        for (unsigned int j=0;j<labels[i].size();j++){
            if (pos.x==labels[i][j].x && pos.y==labels[i][j].y){
                label_pos=i+1;
                found_pos=true;
            }
            if(prev.x>=0 &&  prev.y>=0)
            {
                if(prev.x==labels[i][j].x && prev.y==labels[i][j].y){
                    prev_label=i+1;
                    found_prev=true;
                }
            }
            if ( found_pos && ((found_prev) || (prev.x < 0) || (prev.y < 0) ))
                break;
        }
        if(found_pos && ( (found_prev) || (prev.x<0) || (prev.y<0) ) )
            break;
    }

    for (unsigned int i=0;i<labels.size();i++){
        if( i!=(label_pos-1) ){
            for(unsigned int j=0;j<labels[i].size();j++){
                r_map.at<uchar>(labels[i][j].x,labels[i][j].y)=0;
            }
        }
     }

    return (prev_label!=label_pos) && found_pos; //returns true if reachable set changes from prev position
}


bool VisC_transf::valid_pos(cv::Point3i pos)
{
    bool value=(map_erosionOp.at<uchar>(pos.x,pos.y)!=0);
    if(!value)
    {
        map_debug=cv::Mat::zeros(map_or.rows, map_or.cols, CV_8UC1);
        act_dist=cv::Mat_<int>::zeros(map_or.rows, map_or.cols);
    }
    return value;
}

cv::Mat_<int> distance_transform(cv::Mat r_map, cv::Point pos){
    cv::Mat_<int> ret=cv::Mat_<int>::ones(r_map.rows, r_map.cols)*-1;
    if(pos.x<0 || pos.x>=r_map.rows || pos.y<0 || pos.y>=r_map.cols)
        return ret;
    if(r_map.at<uchar>(pos.x,pos.y)==0)
        return ret;

    vector<cv::Point> vc;
    vc.clear();
    vc.push_back(pos);
    ret(pos.x,pos.y)=0;

    while(vc.size()!=0){
        //cout<<vc[0].x<<" "<<vc[0].y<<" "<<ret(vc[0].x,vc[0].y)<<endl;
        int lx=boundPos(vc[0].x-1,r_map.rows);
        int ux=boundPos(vc[0].x+1,r_map.rows);
        int ly=boundPos(vc[0].y-1,r_map.cols);
        int uy=boundPos(vc[0].y+1,r_map.cols);

        for(int i=lx; i<=ux; i++){
            for(int j=ly; j<=uy; j++){
                if( r_map.at<uchar>(i,j)!=0 && ret(i,j)==-1 ){
                    vc.push_back(cv::Point(i,j));
                    ret(i,j)=ret(vc[0].x,vc[0].y)+1;
                }
            }
        }
        vc.erase(vc.begin());
    }

    return ret;
}

cv::Mat_<int> create_robot_model(int size){
    cv::Mat_<int> ret=cv::Mat_<int>::zeros(2*size+1, 2*size+1);
    for(int i=0;i<ret.rows;i++){
        for(int j=0;j<ret.cols;j++){
            if( round(sqrt((i-size)*(i-size)+(j-size)*(j-size)))<=round((size)) ){
                ret(i,j)=1;
            }
        }
    }
    return ret;
}

cv::Mat_<int> actuation_transform(cv::Mat r_map, cv::Point pos, int size){
    cv::Mat_<int> ret=cv::Mat_<int>::ones(r_map.rows, r_map.cols)*-1;
    if(pos.x<0 || pos.x>=r_map.rows || pos.y<0 || pos.y>=r_map.cols)
        return ret;
    if(r_map.at<uchar>(pos.x,pos.y)==0)
        return ret;

    cv::Mat_<int> reach=cv::Mat_<int>::ones(r_map.rows, r_map.cols)*-1;

    cv::Mat_<int> robot=create_robot_model(size);

    vector<cv::Point> vc;
    vc.clear();
    vc.push_back(pos);
    reach(pos.x,pos.y)=0;


    while(vc.size()!=0){
        //cout<<vc[0].x<<" "<<vc[0].y<<" "<<ret(vc[0].x,vc[0].y)<<endl;
        int lx=boundPos(vc[0].x-size,r_map.rows);
        int ux=boundPos(vc[0].x+size,r_map.rows);
        int ly=boundPos(vc[0].y-size,r_map.cols);
        int uy=boundPos(vc[0].y+size,r_map.cols);

        for(int i=lx; i<=ux; i++){
            for(int j=ly; j<=uy; j++){
                if(robot(i-vc[0].x+size,j-vc[0].y+size) && ret(i,j)==-1 ){
                    ret(i,j)=reach(vc[0].x,vc[0].y)+1;
                }
                if( r_map.at<uchar>(i,j)!=0 && reach(i,j)==-1 && max(abs(i-vc[0].x),abs(j-vc[0].y))<=1 ){
                    vc.push_back(cv::Point(i,j));
                    reach(i,j)=reach(vc[0].x,vc[0].y)+1;
                }
            }
        }
        vc.erase(vc.begin());
    }

    cout<<cv::countNonZero(ret)<<"; Size: "<<ret.rows*ret.cols<<endl;

    return ret;
}

void VisC_transf::visibility(cv::Point3i pos, bool proc, ros::Time t01)
{
    ros::Time ttt;

    ttt=ros::Time::now();

    cv::Mat r_map=map_erosionOp.clone();

    bool new_v=reachability_map(pos,r_map);

    if( (new_v) || (prev.x<0) || (prev.y<0) || proc )  //if visibility is changed
    {
        int rad=min(infl,defl);  //if defl<infl, visibility is given by morphological closing

        //ros::Time ttt;

        //ttt=ros::Time::now();

        cv::Mat_<int> dist_transf=actuation_transform(r_map, cv::Point(pos.x,pos.y), rad);
        //cout<<ros::Time::now()-ttt<<endl;
        double min_, max_;
        cv::minMaxLoc(dist_transf, &min_, &max_);
        cv::Mat dist;
        dist_transf.convertTo(dist, CV_8U , 255 / max_);
        cv::imshow("dist",dist);

        string filename = ros::package::getPath("map_transform").append("/images/dist.png");
        cv::imwrite(filename, dist);

        act_dist=dist_transf;


        cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                               cv::Size( 2*rad + 1, 2*rad+1 ),
                                               cv::Point( rad, rad ) );

        cv::Mat act_map=r_map.clone();

        ttt=ros::Time::now();

        dilate( act_map, act_map, element);  //actuation space

        cout<<ros::Time::now()-ttt<<endl;

        cv::Mat vis_map=act_map.clone();

        Unreachable unreach(map_or, act_map);

        map_debug=unreach.unreach_map;

        if(infl<defl)  //extended sensing radius
        {
            vis_map=ext_vis(unreach, vis_map, r_map, opt);
        }

        map_label=r_map;
        map_act=act_map;
        map_vis=vis_map;

        unsigned char color[3]={0,255,0};

        map_erosionOpPrintColor=printPoint(map_erosionOp, cv::Point(pos.x,pos.y), color);

        ros::Duration diff = ros::Time::now() - t01;

        ROS_INFO("%s - Time for visibility: %f", tf_pref.c_str(), diff.toSec());

        unsigned char c123[3]={0,200,200};
        unsigned char c12[3]={255,255,0};
        unsigned char c13[3]={255,0,255};
        unsigned char c23[3]={0,0,200};
        unsigned char c10[3]={0,255,0};
        unsigned char c20[3]={255,255,255};
        unsigned char c30[3]={150,0,0};
        unsigned char c00[3]={0,0,0};
        unsigned char ctest[3]={18,143,226};

        unsigned char c1[3]={0,150,0};
        unsigned char c2[3]={0,0,0};
        unsigned char c3[3]={0,150,0};
        unsigned char c0[3]={150,0,0};


        cv::Mat_<int> robot_mod=create_robot_model(infl);
        //cout<<ros::Time::now()-ttt<<endl;
        double minR_, maxR_;
        cv::minMaxLoc(robot_mod, &minR_, &maxR_);

        cv::Mat rob;
        robot_mod.convertTo(rob, CV_8U , 255 / maxR_);
        cv::imshow("robot",rob);

        filename = ros::package::getPath("map_transform").append("/images/robot.png");
        cv::imwrite(filename, rob);

        cv::Mat_<int> sensor_mod=create_robot_model(defl+1);
        //cout<<ros::Time::now()-ttt<<endl;
        cv::minMaxLoc(sensor_mod, &minR_, &maxR_);

        cv::Mat sens;
        sensor_mod.convertTo(sens, CV_8U , 255 / maxR_);

        sensor_mod=create_robot_model(defl-1);
        //cout<<ros::Time::now()-ttt<<endl;
        cv::minMaxLoc(sensor_mod, &minR_, &maxR_);

        cv::Mat sens2, sens3;
        sensor_mod.convertTo(sens2, CV_8U , 255 / maxR_);
        cout<<sens.rows<<" "<<sens.cols<<" "<<sens2.rows<<" "<<sens2.cols<<endl;

        //value = Scalar( 0, rng.uniform(0, 255), rng.uniform(0, 255) );
        copyMakeBorder( sens2, sens3, 2, 2, 2, 2, cv::BORDER_CONSTANT, 0 );

        cout<<sens2.rows<<" "<<sens2.cols<<" "<<sens3.rows<<" "<<sens3.cols<<endl;

        cv::imshow("sensor",sens-sens3);

        filename = ros::package::getPath("map_transform").append("/images/sensor.png");
        cv::imwrite(filename, sens-sens3);

        cv::Mat map_debug1=color_print(map_erosionOp, map_or,  c1, c2, c3, c0);

        cv::imshow("CS",map_debug1);
        cv::waitKey(3);

        filename = ros::package::getPath("map_transform").append("/images/cs.png");
        cv::imwrite(filename, map_debug1);

        cv::Mat map_debug2=color_print3(map_erosionOp, map_closeOp, map_or, c123, c12, c13, c23, c10, c20, c30, c00 );

        cv::imshow("CO",map_debug2);
        cv::waitKey(3);

        filename = ros::package::getPath("map_transform").append("/images/mc.png");
        cv::imwrite(filename, map_debug2);


        cv::Mat map_debug3=color_print3(map_label, map_act, map_or, c123, c12, c13, c23, c10, c20, c30, c00 );

        cv::imshow("AS",map_debug3);
        cv::waitKey(3);

        filename = ros::package::getPath("map_transform").append("/images/act.png");
        cv::imwrite(filename, map_debug3);

        cv::Mat map_debug4=color_print3(map_vis, map_act, map_or, c20, c12, ctest, c23, c10, c20, c00, c00 );

        cv::imshow("VS",map_debug4);
        cv::waitKey(3);

        filename = ros::package::getPath("map_transform").append("/images/vis.png");
        cv::imwrite(filename, map_debug4);



        if(gt)
        {
            ////////////////////////////

            ros::Time t3;

            t3=ros::Time::now();

            vector<cv::Point> reach_list;
            reach_list.clear();

            for(int i=0; i<r_map.rows;i++)
            {
                for(int j=0; j<r_map.cols;j++)
                {
                    if(r_map.at<uchar>(i,j)!=0)
                        reach_list.push_back(cv::Point(i,j));
                }
            }

            vector<cv::Point> labels_gt=label_seed(map_or.clone()/255,4,cv::Point(pos.x,pos.y));

            cv::Mat eff_gt=cv::Mat::zeros(map_or.rows,map_or.cols, CV_8UC1);

            for(unsigned int j=0;j<labels_gt.size();j++){
                   eff_gt.at<uchar>(labels_gt[j].x,labels_gt[j].y)=255;
            }

            map_debug=eff_gt.clone();

//            t3=ros::Time::now();

//            map_truth=brute_force(map_or, map_label,defl, true, map_act);

//            diff = ros::Time::now() - t3;

//            ROS_INFO("%s - Time for  Optimized brute force sq with act: %f", tf_pref.c_str(), diff.toSec());

//            cv::imshow("TestSQwA",this->map_truth);
//            cv::waitKey(3);

//            ////////////////////////////

//            t3=ros::Time::now();

//            map_truth=brute_force(map_or, map_label,defl, true);

//            diff = ros::Time::now() - t3;

//            ROS_INFO("%s - Time for  Optimized brute force sq without act: %f", tf_pref.c_str(), diff.toSec());

//            cv::imshow("TestSQ",this->map_truth);
//            cv::waitKey(3);

//            ////////////////////////////

//            t3=ros::Time::now();

//            map_truth=brute_force(map_or, map_label,defl, false, map_act);

//            diff = ros::Time::now() - t3;

//            ROS_INFO("%s - Time for  Optimized brute force grid with act: %f", tf_pref.c_str(), diff.toSec());

//            cv::imshow("TestGwA",this->map_truth);
//            cv::waitKey(3);

//            ////////////////////////////

//            t3=ros::Time::now();

//            map_truth=brute_force(map_or, map_label,defl, false);

//            diff = ros::Time::now() - t3;

//            ROS_INFO("%s - Time for  Optimized brute force grid without act: %f", tf_pref.c_str(), diff.toSec());

//            cv::imshow("TestG",this->map_truth);
//            cv::waitKey(3);

//            ////////////////////////////

            //t3=ros::Time::now();

            map_truth=brute_force(eff_gt, reach_list,defl, false, map_act,true, true);
            //map_truth=brute_force(map_or, reach_list,defl, false, map_act,true, true);

            cout<<"VM: "<<(unsigned int)map_vis.at<uchar>(182,11)<<"; GT: "<<(unsigned int)map_truth.at<uchar>(182,11)<<endl;

            diff = ros::Time::now() - t3;

            ROS_INFO("%s - Time for  Optimized brute force list with act: %f", tf_pref.c_str(), diff.toSec());

            //cv::imshow("TestLwA",this->map_truth);
            //cv::waitKey(3);

            unsigned char c1[3]={255,255,255};
            unsigned char c2[3]={0,0,0};
            unsigned char c3[3]={255,0,0};
            unsigned char c0[3]={0,180,255};

            cv::Mat comp=color_print(map_vis, map_truth,  c1, c2, c3, c0);

            //cv::Mat upd[3];

            //cv::split(comp,upd);

            //upd[0].at<uchar>(182,11)=0;
            //upd[1].at<uchar>(182,11)=255;
            //upd[2].at<uchar>(182,11)=0;

            //cv::merge(upd,3,comp);

            map_comp=comp.clone();

            //cv::imshow("Comparison",comp);
            //cv::waitKey(3);

            ROS_INFO("Precision: 1; Recall: %f", coverage(map_act,map_truth));

            vector<int> cm=confusion_matrix(map_vis,map_truth);

            float tt=(float)(cm[0]+cm[1]+cm[2]+cm[3]);

            if(cm.size()!=0)
                ROS_INFO("Confusion Matrix:\n     Truth\n     P      N\nV  P %.4f %.4f\nI\nS  N %.4f %.4f", cm[0]/tt,cm[1]/tt,cm[2]/tt,cm[3]/tt);

            ROS_INFO("Precision: %f; Recall: %f",cm[0]/float(cm[0]+cm[1]),cm[0]/float(cm[0]+cm[2]));

            ////////////////////////////

//            t3=ros::Time::now();

//            map_truth=brute_force(map_or, reach_list,defl, false);

//            diff = ros::Time::now() - t3;

//            ROS_INFO("%s - Time for  Optimized brute force list without act: %f", tf_pref.c_str(), diff.toSec());

//            cv::imshow("TestL",this->map_truth);
//            cv::waitKey(3);

//            ////////////////////////////////

            gt_c=true;
        }
    }
}

cv::Mat VisC_transf::ext_vis(Unreachable unreach, cv::Mat vis_map, cv::Mat r_map, bool optRay)
{
    unreach.getFrontiers();

    cv::Mat regions=unreach.regions;

    cv::Mat vis_map_temp;

    CritPoints critP(map_or, r_map, infl);

    //vis_.assign(vis_map.rows*vis_map.cols, map_transform::VisNode());
    //vis_.assign(vis_map.rows*vis_map.cols, -2);
    vis_.vis.assign(vis_map.rows*vis_map.cols, -2);
    vis_.crit_points.assign(vis_map.rows*vis_map.cols,map_transform::VisNode());


    for (unsigned int k=0;k<unreach.frontiers.size();k++){//1;k++){//
        long int countP=0;

        for(unsigned int ff=0;ff<unreach.frontiers[k].size();ff++)
        {
            vector<cv::Point> frontier=unreach.frontiers[k][ff];

            if(frontier.size()>0)
            {
                cv::Point2i crit=critP.find_crit_point(frontier);

                if(!critP.valid())
                {
                    cout<<"Here!"<<endl;
                    continue;
                }
                else
                {
                    critP.frontier_extremes();

                    //// TODO:neighbor points

                    int n_w=0;
                    cv::Point2i critP0, critP1;

                    while(n_w>=0 && unreach.pixel_count[k]!=countP)
                    {
                        n_w++;

                        cv::Point2i crit_point=crit;
                        vector<float> extremes=critP.getExtremes();
                        unsigned int obt=critP.getObt();

                        //cout<<"Region: "<<k<<"; Frontier: "<<ff<<"; Attempt: "<<n_w<<endl;

                        if(n_w==1)
                        {
                           map_debug.at<uchar>(crit.x, crit.y)=0;
                           n_w=-1;
                        }
                        else if((n_w==2 || n_w==3) && critP.getExtremes().size()==2)
                        {
                            if(n_w==2)
                            {
                                float angleC2F;

                                if(critP.getObt()==0)
                                    angleC2F=PI-(critP.getExtremes()[0]+critP.getExtremes()[1])/2;
                                else
                                    angleC2F=(critP.getExtremes()[0]+critP.getExtremes()[1])/2;

                               // if(k==1)
                               //     cout<<angleC2F<<endl;

                                int min_x=min(critP.getExtremesP()[0].x,critP.getExtremesP()[1].x);
                                int min_y=min(critP.getExtremesP()[0].y,critP.getExtremesP()[1].y);
                                int max_x=max(critP.getExtremesP()[0].x,critP.getExtremesP()[1].x);
                                int max_y=max(critP.getExtremesP()[0].y,critP.getExtremesP()[1].y);

                                FindMin<float, cv::Point2i> crit_0,crit_1;

                                for(int x=max(min_x-infl,0);x<min(max_x+infl,r_map.rows);x++)
                                {
                                    for(int y=max(min_y-infl,0);y<min(max_y+infl,r_map.cols);y++)
                                    {
                                        if(r_map.at<uchar>(x,y)==255)
                                        {
                                            //if(k==1)
                                            //    cout<<x<<"; "<<y<<endl;
                                            float angleT=atan2(critP.getExtremesP()[0].y-y,critP.getExtremesP()[0].x-x);
                                            float distT=((critP.getExtremesP()[0].y-y)*(critP.getExtremesP()[0].y-y)+(critP.getExtremesP()[0].x-x)*(critP.getExtremesP()[0].x-x))/infl/infl;
                                            crit_0.iter(abs(boundAngleRN(angleT-angleC2F))+distT,cv::Point2i(x,y));

                                            //if(k==1)
                                            //    cout<<angleT<<endl;

                                            //if(k==1)
                                            //    cout<<abs(boundAngleRN(angleT-angleC2F))<<"; "<<boundAngleRN(angleT-angleC2F)<<endl;


                                            angleT=atan2(critP.getExtremesP()[1].y-y,critP.getExtremesP()[1].x-x);
                                            distT=((critP.getExtremesP()[1].y-y)*(critP.getExtremesP()[1].y-y)+(critP.getExtremesP()[1].x-x)*(critP.getExtremesP()[1].x-x))/infl/infl;

                                            crit_1.iter(abs(boundAngleRN(angleT-angleC2F))+distT,cv::Point2i(x,y));


                                        }
                                    }
                                }

                                critP0=crit_0.getP();
                                critP1=crit_1.getP();

                                crit_point=critP0;

                                float angle0=atan2(critP.getExtremesP()[0].y-crit_point.y,critP.getExtremesP()[0].x-crit_point.x);
                                float angle1=atan2(critP.getExtremesP()[1].y-crit_point.y,critP.getExtremesP()[1].x-crit_point.x);

                                if(angle0<=angle1)
                                {
                                    extremes[0]=angle0;
                                    extremes[1]=angle1;
                                }
                                else
                                {
                                    extremes[0]=angle1;
                                    extremes[1]=angle0;
                                }

                                if( (extremes[1]-extremes[0])>PI )
                                    obt=0;
                                else
                                    obt=1;
                            }
                            else if(n_w==3)
                            {
                                crit_point=critP1;

                                float angle0=atan2(critP.getExtremesP()[0].y-crit_point.y,critP.getExtremesP()[0].x-crit_point.x);
                                float angle1=atan2(critP.getExtremesP()[1].y-crit_point.y,critP.getExtremesP()[1].x-crit_point.x);

                                if(angle0<=angle1)
                                {
                                    extremes[0]=angle0;
                                    extremes[1]=angle1;
                                }
                                else
                                {
                                    extremes[0]=angle1;
                                    extremes[1]=angle0;
                                }

                                if( (extremes[1]-extremes[0])>PI )
                                    obt=0;
                                else
                                    obt=1;
                            }
                        }
                        else
                        {
                            n_w=-1;
                            continue;
                        }

                        vis_map_temp = cv::Mat::zeros(regions.rows, regions.cols, CV_8UC1)*255;

                        vector<cv::Point> occ=expVisibility_obs(crit_point, defl, regions, k, extremes, obt, vis_map_temp);



                        if(optRay)
                        {
                            vector<cv::Point> occ_crit_filt=getExtremeFromObstacles(occ, crit_point);

                            for(unsigned int c=0;c<occ_crit_filt.size();c++)
                            {
                                raytracing(&vis_map_temp, cv::Point2i(crit_point.x,crit_point.y), occ_crit_filt[c], occ_crit_filt[c], defl);
                            }

                            if(k==1)
                            {
                                if(n_w==1)
                                {
                                    //cout<<"Crit: "<<crit_point.x<<"; "<<crit_point.y<<"; Estremes: "<<extremes[0]<<"; "<<extremes[1]<<"; Obtuse"<<obt<<endl;
                                    //cv::imshow("Ray",vis_map_temp);
                                }
                                else if(n_w==2)
                                {
                                    //cout<<"Crit: "<<crit_point.x<<"; "<<crit_point.y<<"; Estremes: "<<extremes[0]<<"; "<<extremes[1]<<"; Obtuse"<<obt<<endl;
                                    //cv::imshow("Ray0",vis_map_temp);
                                }
                                else if(n_w==3)
                                {
                                    //cout<<"Crit: "<<crit_point.x<<"; "<<crit_point.y<<"; Estremes: "<<extremes[0]<<"; "<<extremes[1]<<"; Obtuse"<<obt<<endl;
                                    //cv::imshow("Ray1",vis_map_temp);
                                }
                                //cv::waitKey(3);
                            }

                            for(unsigned int j=0;j<frontier.size();j++){
                                if(vis_map_temp.at<uchar>( frontier[j].x,frontier[j].y)==255)
                                {
                                    std::vector<cv::Point> points_vis=label_seed(vis_map_temp.clone()/255,4,cv::Point(frontier[j].x,frontier[j].y));
                                    for(unsigned int pv=0;pv<points_vis.size();pv++)
                                    {
                                        if(vis_map.at<uchar>(points_vis[pv].x,points_vis[pv].y)!=255)
                                            countP++;

                                        vis_map.at<uchar>(points_vis[pv].x,points_vis[pv].y)=255;

                                        vis_map_temp.at<uchar>(points_vis[pv].x,points_vis[pv].y)=0;

                                        geometry_msgs::Pose pcp;
                                        pcp.position.x=points_vis[pv].x-crit_point.x;
                                        pcp.position.y=points_vis[pv].y-crit_point.y;
                                        float diff=sqrt(pcp.position.x*pcp.position.x+pcp.position.y*pcp.position.y);
                                        if(vis_.vis[points_vis[pv].x*vis_map.cols+points_vis[pv].y]<0)
                                        {
                                            vis_.vis[points_vis[pv].x*vis_map.cols+points_vis[pv].y]=diff;

                                            vis_.crit_points[points_vis[pv].x*vis_map.cols+points_vis[pv].y].points.clear();
                                            geometry_msgs::Pose ps;
                                            ps.position.x=crit_point.x;
                                            ps.position.y=crit_point.y;
                                            vis_.crit_points[points_vis[pv].x*vis_map.cols+points_vis[pv].y].points.push_back(ps);
                                        }
                                        else
                                        {
                                            vis_.vis[points_vis[pv].x*vis_map.cols+points_vis[pv].y]=min(vis_.vis[points_vis[pv].x*vis_map.cols+points_vis[pv].y],diff);//points.push_back(pcp);

                                            geometry_msgs::Pose ps;
                                            ps.position.x=crit_point.x;
                                            ps.position.y=crit_point.y;
                                            vis_.crit_points[points_vis[pv].x*vis_map.cols+points_vis[pv].y].points.push_back(ps);
                                        }
                                    }
                                    //break;
                                }
                            }
                        }
                        else
                        {
                            //cout<<"Here: "<<crit_point.x<<"; "<<crit_point.y<<endl;
                            vis_map_temp=bf_pt(map_or, crit_point, defl, vis_map_temp, true, true);

                            if(k==1)
                            {
                                if(n_w==1)
                                {
                                    //cout<<"Crit: "<<crit_point.x<<"; "<<crit_point.y<<"; Estremes: "<<extremes[0]<<"; "<<extremes[1]<<"; Obtuse"<<obt<<endl;
                                    //cv::imshow("Ray",vis_map_temp);
                                }
                                else if(n_w==2)
                                {
                                    //cout<<"Crit: "<<crit_point.x<<"; "<<crit_point.y<<"; Estremes: "<<extremes[0]<<"; "<<extremes[1]<<"; Obtuse"<<obt<<endl;
                                    //cv::imshow("Ray0",vis_map_temp);
                                }
                                else if(n_w==3)
                                {
                                    //cout<<"Crit: "<<crit_point.x<<"; "<<crit_point.y<<"; Estremes: "<<extremes[0]<<"; "<<extremes[1]<<"; Obtuse"<<obt<<endl;
                                    //cv::imshow("Ray1",vis_map_temp);
                                }
                                cv::waitKey(3);
                            }

                            for(int xx=0;xx<vis_map_temp.rows;xx++)
                            {
                                for(int yy=0;yy<vis_map_temp.cols;yy++)
                                {
                                    if(vis_map_temp.at<uchar>(xx,yy)==255)
                                    {
                                        if(vis_map.at<uchar>(xx,yy)!=255)
                                            countP++;

                                        vis_map.at<uchar>(xx,yy)=255;

                                        countP++;

                                        geometry_msgs::Pose pcp;
                                        pcp.position.x=xx-crit_point.x;
                                        pcp.position.y=yy-crit_point.y;
                                        float diff=sqrt(pcp.position.x*pcp.position.x+pcp.position.y*pcp.position.y);
                                        if(vis_.vis[xx*vis_map.cols+yy]<0)
                                        {
                                            vis_.vis[xx*vis_map.cols+yy]=diff;

                                            vis_.crit_points[xx*vis_map.cols+yy].points.clear();
                                            geometry_msgs::Pose ps;
                                            ps.position.x=crit_point.x;
                                            ps.position.y=crit_point.y;
                                            vis_.crit_points[xx*vis_map.cols+yy].points.push_back(ps);
                                        }
                                        else
                                        {
                                            vis_.vis[xx*vis_map.cols+yy]=min(vis_.vis[xx*vis_map.cols+yy],diff);//points.push_back(pcp);

                                            geometry_msgs::Pose ps;
                                            ps.position.x=crit_point.x;
                                            ps.position.y=crit_point.y;
                                            vis_.crit_points[xx*vis_map.cols+yy].points.push_back(ps);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if(optRay)
    {
        ROS_INFO("Optimized");
    }
    else
    {
        ROS_INFO("Ray Casting");
    }

    return vis_map;
}


void VisC_transf::clearImgs(void)
{
    map_debug=cv::Mat::zeros(map_or.rows, map_or.cols, CV_8UC1);
    map_erosionOpPrintColor=map_debug;
    map_label=map_debug;
    map_act=map_debug;
    map_vis=map_debug;
    map_truth=map_debug;
    act_dist=cv::Mat_<int>::zeros(map_or.rows, map_or.cols);
}
