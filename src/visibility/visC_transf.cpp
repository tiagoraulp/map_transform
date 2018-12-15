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
        _show=config.show;
        gt=config.ground_truth;
        pub_once=config.pub_once;
        frga=config.frga;
        city=config.city;
        fast_opt=config.fast_opt;
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
    nh_.param("fast_opt", fast_opt, false);
    graph_publisher = nh_.advertise<map_transform::VisCom>("graph", 10,true);
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
}


VisC_transf::~VisC_transf()
{
    if(_show){
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
    if(count>0 && _show){
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

    if(!_show)
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

void thinningIteration(cv::Mat& im, int iter)
{
    cv::Mat marker = cv::Mat::zeros(im.size(), CV_8UC1);

    for (int i = 1; i < im.rows-1; i++)
    {
        for (int j = 1; j < im.cols-1; j++)
        {
            uchar p2 = im.at<uchar>(i-1, j);
            uchar p3 = im.at<uchar>(i-1, j+1);
            uchar p4 = im.at<uchar>(i, j+1);
            uchar p5 = im.at<uchar>(i+1, j+1);
            uchar p6 = im.at<uchar>(i+1, j);
            uchar p7 = im.at<uchar>(i+1, j-1);
            uchar p8 = im.at<uchar>(i, j-1);
            uchar p9 = im.at<uchar>(i-1, j-1);

            int A  = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) +
                     (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) +
                     (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                     (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
            int B  = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
            int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

            if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                marker.at<uchar>(i,j) = 1;
        }
    }

    im &= ~marker;
}


cv::Mat skel3(cv::Mat img) // zhang suen thinning
{
    bitwise_not(img,img);
    img /= 255;

    cv::Mat prev = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::Mat diff;

    do {
        thinningIteration(img, 0);
        thinningIteration(img, 1);
        cv::absdiff(img, prev, diff);
        img.copyTo(prev);
    }
    while (cv::countNonZero(diff) > 0);

    img *= 255;
    bitwise_not(img,img);

    return img;
}

void ThinSubiteration1(cv::Mat & pSrc, cv::Mat & pDst) {
    int rows = pSrc.rows;
    int cols = pSrc.cols;
    pSrc.copyTo(pDst);
    for(int i = 0; i < rows; i++) {
            for(int j = 0; j < cols; j++) {
                    if(pSrc.at<float>(i, j) == 1.0f) {
                            /// get 8 neighbors
                            /// calculate C(p)
                            int neighbor0 = (int) pSrc.at<float>( i-1, j-1);
                            int neighbor1 = (int) pSrc.at<float>( i-1, j);
                            int neighbor2 = (int) pSrc.at<float>( i-1, j+1);
                            int neighbor3 = (int) pSrc.at<float>( i, j+1);
                            int neighbor4 = (int) pSrc.at<float>( i+1, j+1);
                            int neighbor5 = (int) pSrc.at<float>( i+1, j);
                            int neighbor6 = (int) pSrc.at<float>( i+1, j-1);
                            int neighbor7 = (int) pSrc.at<float>( i, j-1);
                            int C = int(~neighbor1 & ( neighbor2 | neighbor3)) +
                                             int(~neighbor3 & ( neighbor4 | neighbor5)) +
                                             int(~neighbor5 & ( neighbor6 | neighbor7)) +
                                             int(~neighbor7 & ( neighbor0 | neighbor1));
                            if(C == 1) {
                                    /// calculate N
                                    int N1 = int(neighbor0 | neighbor1) +
                                                     int(neighbor2 | neighbor3) +
                                                     int(neighbor4 | neighbor5) +
                                                     int(neighbor6 | neighbor7);
                                    int N2 = int(neighbor1 | neighbor2) +
                                                     int(neighbor3 | neighbor4) +
                                                     int(neighbor5 | neighbor6) +
                                                     int(neighbor7 | neighbor0);
                                    int N = cv::min(N1,N2);
                                    if ((N == 2) || (N == 3)) {
                                            /// calculate criteria 3
                                            int c3 = ( neighbor1 | neighbor2 | ~neighbor4) & neighbor3;
                                            if(c3 == 0) {
                                                    pDst.at<float>( i, j) = 0.0f;
                                            }
                                    }
                            }
                    }
            }
    }
}


void ThinSubiteration2(cv::Mat & pSrc, cv::Mat & pDst) {
    int rows = pSrc.rows;
    int cols = pSrc.cols;
    pSrc.copyTo( pDst);
    for(int i = 0; i < rows; i++) {
            for(int j = 0; j < cols; j++) {
                    if (pSrc.at<float>( i, j) == 1.0f) {
                            /// get 8 neighbors
                            /// calculate C(p)
                        int neighbor0 = (int) pSrc.at<float>( i-1, j-1);
                        int neighbor1 = (int) pSrc.at<float>( i-1, j);
                        int neighbor2 = (int) pSrc.at<float>( i-1, j+1);
                        int neighbor3 = (int) pSrc.at<float>( i, j+1);
                        int neighbor4 = (int) pSrc.at<float>( i+1, j+1);
                        int neighbor5 = (int) pSrc.at<float>( i+1, j);
                        int neighbor6 = (int) pSrc.at<float>( i+1, j-1);
                        int neighbor7 = (int) pSrc.at<float>( i, j-1);
                            int C = int(~neighbor1 & ( neighbor2 | neighbor3)) +
                                    int(~neighbor3 & ( neighbor4 | neighbor5)) +
                                    int(~neighbor5 & ( neighbor6 | neighbor7)) +
                                    int(~neighbor7 & ( neighbor0 | neighbor1));
                            if(C == 1) {
                                    /// calculate N
                                    int N1 = int(neighbor0 | neighbor1) +
                                            int(neighbor2 | neighbor3) +
                                            int(neighbor4 | neighbor5) +
                                            int(neighbor6 | neighbor7);
                                    int N2 = int(neighbor1 | neighbor2) +
                                            int(neighbor3 | neighbor4) +
                                            int(neighbor5 | neighbor6) +
                                            int(neighbor7 | neighbor0);
                                    int N = cv::min(N1,N2);
                                    if((N == 2) || (N == 3)) {
                                            int E = (neighbor5 | neighbor6 | ~neighbor0) & neighbor7;
                                            if(E == 0) {
                                                    pDst.at<float>(i, j) = 0.0f;
                                            }
                                    }
                            }
                    }
            }
    }
}


cv::Mat skel2(cv::Mat img){ // another zhang suen algorithm for thinning
    bool bDone = false;

    bitwise_not(img,img);

    int rows = img.rows;
    int cols = img.cols;

    /// start to thin
    cv::Mat p_thinMat1 = cv::Mat::zeros(rows + 2, cols + 2, CV_32FC1);
    cv::Mat p_thinMat2 = cv::Mat::zeros(rows + 2, cols + 2, CV_32FC1);
    cv::Mat p_cmp = cv::Mat::zeros(rows + 2, cols + 2, CV_8UC1);

    while (bDone != true) {
            /// sub-iteration 1
            ThinSubiteration1(img, p_thinMat1);
            /// sub-iteration 2
            ThinSubiteration2(p_thinMat1, p_thinMat2);
            /// compare
            cv::compare(img, p_thinMat2, p_cmp, CV_CMP_EQ);
            /// check
            int num_non_zero = countNonZero(p_cmp);
            if(num_non_zero == (rows + 2) * (cols + 2)) {
                    bDone = true;
            }
            /// copy
            p_thinMat2.copyTo(img);
    }

    bitwise_not(img,img);

    return img;
}

cv::Mat skel(cv::Mat img)
{
    bitwise_not(img,img);

    cv::threshold(img, img, 127, 255, cv::THRESH_BINARY);
    cv::Mat skel(img.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat temp;
    cv::Mat eroded;

    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    bool done;
    do
    {
      cv::erode(img, eroded, element);
      cv::dilate(eroded, temp, element); // temp = open(img)
      cv::subtract(img, temp, temp);
      cv::bitwise_or(skel, temp, skel);
      eroded.copyTo(img);

      done = (cv::countNonZero(img) == 0);
    } while (!done);

    bitwise_not(skel,skel);

    return skel;
}


cv::Mat closeWithSkel(cv::Mat img, int gap){
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(gap, gap));
    cv::Mat cskel;
    cv::erode(img.clone(), cskel, element);

    cskel=skel3(cskel);
    cv::bitwise_not(cskel,cskel);
    cv::Mat img_rev;
    cv::bitwise_not(img.clone(),img_rev);
    cv::bitwise_or(cskel, img_rev, cskel);
    cv::bitwise_not(cskel, cskel);

    // Test - subtract closed


    return cskel;
}

bool VisC_transf::conf_space(void)
{
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                           cv::Size( 2*infl + 1, 2*infl+1 ),
                                           cv::Point( infl, infl ) );

    cv::Mat element_d = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                           cv::Size( 2*infl + 1, 2*infl+1 ),
                                           cv::Point( infl, infl ) );

    cv::Mat or_map, or_map_perc, er_map, cr_map;

    or_map=cv_map_scaled.clone();
    or_map_perc=cv_map_scaled_perc.clone();
    //or_map=closeWithSkel(or_map, 8);

    cv::imshow("Map_Original",cv_map_scaled.clone());
    cv::waitKey(3);
    cv::imshow("Map_skeleton",or_map);
    cv::waitKey(3);

    msg_rcv_pub=msg_rcv;

    cv::erode( or_map, er_map, element);//,cv::Point(-1,-1),1,cv::BORDER_CONSTANT,0);//cv::morphologyDefaultBorderValue());
    cv::dilate( er_map, cr_map, element_d);

    map_or=or_map;
    map_or_perc=or_map_perc;
    map_erosionOp=er_map;
    map_erosionOpSmall=map_erosionOp;
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
            if( ((i-size)*(i-size)+(j-size)*(j-size))<=(size*size) ){
            //if( round(sqrt((i-size)*(i-size)+(j-size)*(j-size)))<=round((size)) ){
                ret(i,j)=1;
            }
        }
    }
    return ret;
}

cv::Mat_<int> actuation_transform(cv::Mat r_map, cv::Point pos, int size, vector<vector<vector<cv::Point> > > & result_act, bool save_act_points=false){
    cv::Mat_<int> ret=cv::Mat_<int>::ones(r_map.rows, r_map.cols)*-1;
    if(save_act_points)
        result_act=vector<vector<vector<cv::Point> > >(r_map.rows, vector<vector<cv::Point> >(r_map.cols, vector<cv::Point>(0)));
    if(pos.x<0 || pos.x>=r_map.rows || pos.y<0 || pos.y>=r_map.cols)
        return ret;
    if(r_map.at<uchar>(pos.x,pos.y)==0)
        return ret;

    cv::Mat_<int> reach=cv::Mat_<int>::ones(r_map.rows, r_map.cols)*-1;

    cv::Mat_<int> robot=create_robot_model(size);

    deque<cv::Point> vc;
    vc.clear();
    vc.push_back(pos);
    reach(pos.x,pos.y)=0;
    if(save_act_points)
        result_act[pos.x][pos.y].push_back(cv::Point(pos.x,pos.y));

    while(vc.size()!=0){
        cv::Point vcpt=vc.front();
        int lx=boundPos(vcpt.x-size,r_map.rows);
        int ux=boundPos(vcpt.x+size,r_map.rows);
        int ly=boundPos(vcpt.y-size,r_map.cols);
        int uy=boundPos(vcpt.y+size,r_map.cols);

        for(int i=lx; i<=ux; i++){
            for(int j=ly; j<=uy; j++){
                if(robot(i-vcpt.x+size,j-vcpt.y+size) && ret(i,j)==-1 ){
                    ret(i,j)=reach(vcpt.x,vcpt.y)+1;
                }
                if( r_map.at<uchar>(i,j)!=0 && reach(i,j)==-1 && max(abs(i-vcpt.x),abs(j-vcpt.y))<=1 ){
                    vc.push_back(cv::Point(i,j));
                    reach(i,j)=reach(vcpt.x,vcpt.y)+1;
                    if(save_act_points)
                        result_act[i][j].push_back(cv::Point(i,j));
                }
                if( robot(i-vcpt.x+size,j-vcpt.y+size) && r_map.at<uchar>(i,j)==0 ){
                    if(save_act_points)
                        result_act[i][j].push_back(cv::Point(vcpt.x,vcpt.y));
                }
            }
        }
        vc.pop_front();
    }
    return ret;
}




void VisC_transf::visibility(cv::Point3i pos, bool proc, ros::Time t01)
{
    ros::Time ttt;

    ttt=ros::Time::now();

    cv::Mat r_map=map_erosionOp.clone();

    ros::Time tt;

    tt=ros::Time::now();

    bool new_v=reachability_map(pos,r_map);

    ros::Duration diff_tt = ros::Time::now() - tt;

    small_frontiers.clear();

    //defl=infl;

    if( (new_v) || (prev.x<0) || (prev.y<0) || proc )  //if visibility is changed
    {
        //cout<<tf_pref.c_str()<<"Time Reach: "<<diff_tt<<endl;

        int rad=min(infl,defl);  //if defl<infl, visibility is given by morphological closing

        //ros::Time ttt;

        //ttt=ros::Time::now();

        vis_.vis.assign(map_or.rows*map_or.cols, -2);
        vis_.crit_points.assign(map_or.rows*map_or.cols,map_transform::VisNode());

        //cv::Mat_<int> distance_transf=distance_transform(r_map, cv::Point(pos.x,pos.y));

        tt=ros::Time::now();
        vector<vector<vector<cv::Point> > > result_act;
        cv::Mat_<int> dist_transf=actuation_transform(r_map, cv::Point(pos.x,pos.y), rad, result_act, frga);
        //cout<<tf_pref.c_str()<<"Time Act_Transf: "<<ros::Time::now()-tt<<endl;

        if(frga){
            geometry_msgs::Pose ps;
            for(auto i=0u; i<result_act.size(); i++){
                for(auto j=0u; j<result_act[i].size(); j++){
                    if(result_act[i][j].size()==1){
                        ps.position.x=result_act[i][j][0].x;
                        ps.position.y=result_act[i][j][0].y;
                        vis_.crit_points[i*map_or.cols+j].points.push_back(ps);
                    }
                    else if(result_act[i][j].size()>1){
                        vector<vector<cv::Point> > results_act=cluster_points(result_act[i][j]);
                        for(auto k=0u; k<results_act.size(); k++){
                            FindMin<float> closest_pt;
                            for(auto l=0u; l<results_act[k].size(); l++){
                                float d1=(float)(i-results_act[k][l].x);
                                float d2=(float)(j-results_act[k][l].y);
                                closest_pt.iter( d1*d1+d2*d2 );
                            }
                            ps.position.x=results_act[k][closest_pt.getInd()].x;
                            ps.position.y=results_act[k][closest_pt.getInd()].y;
                            vis_.crit_points[i*map_or.cols+j].points.push_back(ps);
                        }
                    }
                }
            }
        }

        double min_, max_;
        cv::minMaxLoc(dist_transf, &min_, &max_);
        cv::Mat dist;
        dist_transf.convertTo(dist, CV_8U , 255 / max_);

        act_dist=dist_transf;


        tt=ros::Time::now();
        cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                               cv::Size( 2*rad + 1, 2*rad+1 ),
                                               cv::Point( rad, rad ) );

        cv::Mat act_map=r_map.clone();

        dilate( act_map, act_map, element);  //actuation space

        //cout<<tf_pref.c_str()<<"Time Dilation: "<<ros::Time::now()-tt<<endl;

        vis_map=act_map.clone();

        tt=ros::Time::now();

        Unreachable unreach(map_or_perc, act_map);
        Unreachable unreachCP(map_or, act_map);

        //cout<<tf_pref.c_str()<<"Time Unreach: "<<ros::Time::now()-tt<<endl;

        map_debug=unreach.unreach_map;

        map_label=r_map;
        map_labelSmall=map_label;
        map_act=act_map;

        tt=ros::Time::now();
        if(infl<defl)  //extended sensing radius
        {
            ext_vis(unreach, unreachCP, r_map, opt);
        }
        //cout<<tf_pref.c_str()<<"Time Ext_Vis: "<<ros::Time::now()-tt<<endl;

        map_vis=vis_map;

        unsigned char color[3]={0,255,0};

        map_erosionOpPrintColor=printPoint(map_erosionOp, cv::Point(pos.x,pos.y), color);

        ros::Duration diff = ros::Time::now() - t01;

        ROS_INFO("%s - Time for visibility: %f", tf_pref.c_str(), diff.toSec());

        cv::imshow("dist",dist);

        string filename = ros::package::getPath("map_transform").append("/images/dist.png");
        cv::imwrite(filename, dist);

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
        //cout<<sens.rows<<" "<<sens.cols<<" "<<sens2.rows<<" "<<sens2.cols<<endl;

        //value = Scalar( 0, rng.uniform(0, 255), rng.uniform(0, 255) );
        copyMakeBorder( sens2, sens3, 2, 2, 2, 2, cv::BORDER_CONSTANT, 0 );

        //cout<<sens2.rows<<" "<<sens2.cols<<" "<<sens3.rows<<" "<<sens3.cols<<endl;

        cv::imshow("sensor",sens-sens3);

        filename = ros::package::getPath("map_transform").append("/images/sensor.png");
        cv::imwrite(filename, sens-sens3);

        cv::Mat map_temp=map_or.clone();
        for(unsigned int iii=0; iii<crit_pts.size(); iii++){
            map_temp=printPoint(map_temp, crit_pts[iii], color);
        }
        cv::imshow("CP",map_temp);
        cv::waitKey(3);


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

        cv::Mat map_debug4=color_print3(map_vis, map_act, map_or_perc, c20, c12, ctest, c23, c10, c20, c00, c00 );

        cv::imshow("VS",map_debug4);
        cv::waitKey(3);

        filename = ros::package::getPath("map_transform").append("/images/vis.png");
        cv::imwrite(filename, map_debug4);

        if(vis_temp.rows!=0 && vis_temp.cols!=0){
            cv::Mat map_debug5=color_print3(vis_temp, map_act, map_or_perc, c20, c12, ctest, c23, c10, c20, c00, c00 );

            cv::imshow("VT",map_debug5);
            cv::waitKey(3);
        }

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

            cv::Mat map_gt=map_or_perc.clone();

            for(unsigned int j=0;j<small_frontiers.size();j++){
                   map_gt.at<uchar>(small_frontiers[j].x,small_frontiers[j].y)=0;
            }

            vector<cv::Point> labels_gt=label_seed(map_gt.clone()/255,4,cv::Point(pos.x,pos.y));

            cv::Mat eff_gt=cv::Mat::zeros(map_gt.rows,map_gt.cols, CV_8UC1);

            for(unsigned int j=0;j<labels_gt.size();j++){
                   eff_gt.at<uchar>(labels_gt[j].x,labels_gt[j].y)=255;
            }

            map_debug=eff_gt.clone();

            //cv::imshow("Debug_test", map_debug);
            //cv::waitKey(3);
            //gt_c=true;
            //map_truth=map_debug.clone();
            //map_comp=map_truth;
            //return;

            t3=ros::Time::now();

            map_truth=brute_force(eff_gt, map_label,defl, true, map_act,false, false);
            //map_truth=brute_force(map_gt, map_label,defl, true, map_act);

            diff = ros::Time::now() - t3;

            ROS_INFO("%s - Time for  Optimized brute force sq with act: %f", tf_pref.c_str(), diff.toSec());

//            cv::imshow("TestSQwA",this->map_truth);
//            cv::waitKey(3);

//            ////////////////////////////

//            t3=ros::Time::now();

//            map_truth=brute_force(map_gt, map_label,defl, true);

//            diff = ros::Time::now() - t3;

//            ROS_INFO("%s - Time for  Optimized brute force sq without act: %f", tf_pref.c_str(), diff.toSec());

//            cv::imshow("TestSQ",this->map_truth);
//            cv::waitKey(3);

//            ////////////////////////////

//            t3=ros::Time::now();

//            map_truth=brute_force(map_gt, map_label,defl, false, map_act);

//            diff = ros::Time::now() - t3;

//            ROS_INFO("%s - Time for  Optimized brute force grid with act: %f", tf_pref.c_str(), diff.toSec());

//            cv::imshow("TestGwA",this->map_truth);
//            cv::waitKey(3);

//            ////////////////////////////

//            t3=ros::Time::now();

//            map_truth=brute_force(map_gt, map_label,defl, false);

//            diff = ros::Time::now() - t3;

//            ROS_INFO("%s - Time for  Optimized brute force grid without act: %f", tf_pref.c_str(), diff.toSec());

//            cv::imshow("TestG",this->map_truth);
//            cv::waitKey(3);

//            ////////////////////////////

            //t3=ros::Time::now();

            //map_truth=brute_force(eff_gt, reach_list,defl, false, map_act,false, false);
            //map_truth=brute_force(map_gt, reach_list,defl, false, map_act,false, false);

            cout<<"VM: "<<(unsigned int)map_vis.at<uchar>(182,11)<<"; GT: "<<(unsigned int)map_truth.at<uchar>(182,11)<<endl;

            //diff = ros::Time::now() - t3;

            //ROS_INFO("%s - Time for  Optimized brute force list with act: %f", tf_pref.c_str(), diff.toSec());

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

//            map_truth=brute_force(map_gt, reach_list,defl, false);

//            diff = ros::Time::now() - t3;

//            ROS_INFO("%s - Time for  Optimized brute force list without act: %f", tf_pref.c_str(), diff.toSec());

//            cv::imshow("TestL",this->map_truth);
//            cv::waitKey(3);

//            ////////////////////////////////

            gt_c=true;
        }

        if(pub_once)
            publish();
    }
}

vector<cv::Point> VisC_transf::expVisibility_obs(cv::Point2i crit, int defl, cv::Mat regions, uchar k, vector<float> extremes, unsigned obt_angle, cv::Mat &vis_map_temp, vector<cv::Point> &vis_map_temp_list)
{
    vector<cv::Point> occ;
    for(int rowx=max((crit.x-defl),0);rowx<=min((crit.x+defl),regions.rows-1);rowx++)
    {
        for(int coly=max((crit.y-defl),0);coly<=min((crit.y+defl),regions.cols-1);coly++)
        {
            float angle=atan2(coly-crit.y,rowx-crit.x);
            float dist=(rowx-crit.x)*(rowx-crit.x)+(coly-crit.y)*(coly-crit.y);

            bool reg, free_space;
            if(extremes.size()!=0){
                if(obt_angle==1)
                    reg=(angle<extremes[1] && angle>extremes[0]);
                else
                    reg=(angle<extremes[0] || angle>extremes[1]);

                free_space=( regions.at<uchar>(rowx,coly)==(k+2) );
            } else {
                reg=((map_or_perc.at<uchar>(rowx,coly)==255) && (!(map_act.at<uchar>(rowx,coly)==255)));
                free_space=true;
            }

            if(reg && (dist<=(1*defl*defl)) && free_space )
            {
                if(vis_map.at<uchar>(rowx,coly)==255 && fast_opt){
                    geometry_msgs::Pose pcp;
                    pcp.position.x=rowx-crit.x;
                    pcp.position.y=coly-crit.y;
                    float diff=sqrt(pcp.position.x*pcp.position.x+pcp.position.y*pcp.position.y);
                    if( (vis_.vis[rowx*vis_map.cols+coly]<0) || (diff<(vis_.vis[rowx*vis_map.cols+coly])) )
                    {
                        vis_map_temp.at<uchar>(rowx,coly)=255;
                        vis_map_temp_list.push_back(cv::Point(rowx,coly));
                    }
                } else {
                    vis_map_temp.at<uchar>(rowx,coly)=255;
                    vis_map_temp_list.push_back(cv::Point(rowx,coly));
                }
            }
            else if (reg && (dist<=(1*defl*defl)) && (map_or_perc.at<uchar>(rowx,coly)==0) )
            {
                bool stop=false;
                for(int vx=-1;vx<=1;vx++)
                {
                    for(int vy=-1;vy<=1;vy++)
                    {
                        if( (rowx+vx)>=0 && (rowx+vx)<regions.rows && (coly+vy)>=0 && (coly+vy)<regions.cols )
                        {
                            if( (abs(vx)+abs(vy)==1) &&  regions.at<uchar>(rowx+vx,coly+vy)==(k+2)  )
                            {
                                occ.push_back(cv::Point(rowx,coly));
                                stop=true;
                                break;
                            }
                        }
                    }
                    if(stop)
                        break;
                }
            }
        }
    }

    return occ;
}

cv::Mat VisC_transf::ext_vis(Unreachable unreach, Unreachable unreachCP, cv::Mat r_map, bool optRay)
{
    ros::Time tt=ros::Time::now();
    unreachCP.getFrontiers();
    //cout<<" - time Frontiers: "<<ros::Time::now()-tt<<endl;

    cv::Mat regions=unreach.regions;

    cv::Mat vis_map_temp;
    vector<cv::Point> vis_map_temp_list;

    CritPoints critP(map_or, r_map, infl);

    crit_pts.clear();

    //vis_.assign(vis_map.rows*vis_map.cols, map_transform::VisNode());
    //vis_.assign(vis_map.rows*vis_map.cols, -2);
    //vis_.vis.assign(vis_map.rows*vis_map.cols, -2);
    //vis_.crit_points.assign(vis_map.rows*vis_map.cols,map_transform::VisNode());


    if(!city){
        for (unsigned int k=0;k<unreachCP.frontiers.size();k++){//1;k++){//
            long int countP=0;

            for(unsigned int ff=0;ff<unreachCP.frontiers[k].size();ff++)
            {
                vector<cv::Point> frontier=unreachCP.frontiers[k][ff];

                int frontier_size_threshold=2;

                if(unreachCP.checkFrontierSize(k,ff)>frontier_size_threshold)
                {
                    tt=ros::Time::now();
                    cv::Point2i crit=critP.find_crit_point(frontier);
                    //cout<<" - time Crit Point "<<k<<";"<<ff<<": "<<ros::Time::now()-tt<<endl;

                    if(!critP.valid())
                    {
                        cout<<"Here!"<<endl;
                        continue;
                    }
                    else
                    {

                        tt=ros::Time::now();
                        critP.frontier_extremes();
                        //cout<<" - time Frontier Extremes: "<<ros::Time::now()-tt<<endl;

                        //// TODO:neighbor points

                        int n_w=0;
                        cv::Point2i critP0, critP1;

                        while(n_w>=0 && unreachCP.pixel_count[k]!=countP)
                        {
                            crit_pts.push_back(crit);
                            n_w++;

                            cv::Point2i crit_point=crit;
                            vector<float> extremes=critP.getExtremes();
                            tt=ros::Time::now();
                            unsigned int obt=critP.getObt();
                            //cout<<" - time Determine Obtuse: "<<ros::Time::now()-tt<<endl;

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
                            vis_map_temp_list.clear();

                            uchar k_perc=0;
                            for(int pt_f_k_perc=0;frontier.size();pt_f_k_perc++){
                                if(regions.at<uchar>(frontier[pt_f_k_perc].x,frontier[pt_f_k_perc].y)>1)
                                {
                                    k_perc=regions.at<uchar>(frontier[pt_f_k_perc].x,frontier[pt_f_k_perc].y);
                                    break;
                                }
                            }
                            if(k_perc==0){
                                k_perc=k;
                            }
                            else{
                                k_perc=k_perc-2;
                            }

                            tt=ros::Time::now();
                            vector<cv::Point> occ;
                            if(!two_maps)
                                occ=expVisibility_obs(crit_point, defl, regions, k_perc, extremes, obt, vis_map_temp, vis_map_temp_list);
                            else
                                occ=expVisibility_obs(crit_point, defl, regions, k_perc, vector<float>(0), obt, vis_map_temp, vis_map_temp_list);
                            //cout<<" - time Expetected Visibility: "<<ros::Time::now()-tt<<endl;


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
                                tt=ros::Time::now();
                                //vis_map_temp=bf_pt(map_or_perc, crit_point, defl, vis_map_temp, false, false);
                                vis_map_temp=bf_pt_v2(map_or_perc, crit_point, defl, vis_map_temp, false, false, vis_map_temp_list);
                                //vis_map_temp=bf_pt_v2(map_or_perc, crit_point, defl, vis_map_temp, false, false);
                                //cout<<" - time True Visibility: "<<ros::Time::now()-tt<<endl;

                                if(k==2)
                                    vis_temp=vis_map_temp;

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

                                tt=ros::Time::now();
                                for(int xx=0;xx<vis_map_temp.rows;xx++)
                                {
                                    for(int yy=0;yy<vis_map_temp.cols;yy++)
                                    {
                                        if( (vis_map_temp.at<uchar>(xx,yy)==255) )
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
                                //cout<<" - time Post Processing: "<<ros::Time::now()-tt<<endl;
                            }
                        }
                    }
                }
                else{
                    for(unsigned int pt=0;pt<frontier.size();pt++){
                        small_frontiers.push_back(frontier[pt]);
                    }
                }
            }
        }
    }
    else{
        vector<cv::Point2i> crit_points;
        cv::Point2i crit_point;
        for (unsigned int k=0;k<unreachCP.frontiers.size();k++){//1;k++){//
            for(unsigned int ff=0;ff<unreachCP.frontiers[k].size();ff++)
            {
                vector<cv::Point> frontier=unreachCP.frontiers[k][ff];
                int frontier_size_threshold=2;
                if(unreachCP.checkFrontierSize(k,ff)>frontier_size_threshold)
                {
                    crit_point=critP.find_crit_point(frontier);
                    if(critP.valid() && (r_map.at<uchar>(crit_point.x,crit_point.y)==255))
                        crit_points.push_back(crit_point);
                }
            }
        }
        vector<cv::Point2i> crits_temp;
        for(int r_pt_i=0; r_pt_i<r_map.rows;r_pt_i++){
            for(int r_pt_j=0; r_pt_j<r_map.cols;r_pt_j++){
                if(r_map.at<uchar>(r_pt_i,r_pt_j)==255){
                    FindMin<float,cv::Point2i,cv::Point2i> closest_pt_XYZ;
                    for(auto lct=0u; lct<crits_temp.size(); lct++){
                        float d1=(float)(r_pt_i-crits_temp[lct].x);
                        float d2=(float)(r_pt_j-crits_temp[lct].y);
                        closest_pt_XYZ.iter( d1*d1+d2*d2, crits_temp[lct],cv::Point2i(r_pt_i,r_pt_j) );
                    }
                    for(auto lct=0u; lct<crit_points.size(); lct++){ // it's ok to be closer to real critical points, thus the x2
                        float d1=(float)((r_pt_i-crit_points[lct].x)*2);
                        float d2=(float)((r_pt_j-crit_points[lct].y)*2);
                        closest_pt_XYZ.iter( d1*d1+d2*d2 , crit_points[lct],cv::Point2i(0,0));
                    }
                    if((!closest_pt_XYZ.valid()) || (closest_pt_XYZ.valid() && (closest_pt_XYZ.getVal()>=(3*((float)infl)*3*((float)infl))))){
                        crits_temp.push_back( cv::Point2i(r_pt_i,r_pt_j) );
                    }
                }
            }
        }
        crit_points.insert(crit_points.end(), crits_temp.begin(), crits_temp.end());
        unsigned int number_runs;
        if(fast_opt)
            number_runs=2;
        else
            number_runs=1;
        for(unsigned int run_idx=0;run_idx<number_runs;run_idx++){
            int sens_size;
            if( (run_idx==0) && (fast_opt) )
                sens_size=3*infl;
            else
                sens_size=defl;
            for(unsigned int crit_pt_idx=0;crit_pt_idx<crit_points.size(); crit_pt_idx++){
                crit_point=crit_points[crit_pt_idx];
                crit_pts.push_back(crit_point);
                map_debug.at<uchar>(crit_point.x, crit_point.y)=0;
                vis_map_temp = cv::Mat::zeros(regions.rows, regions.cols, CV_8UC1)*255;
                vis_map_temp_list.clear();
                expVisibility_obs(crit_point, sens_size, map_or_perc, 255, vector<float>(0), 0, vis_map_temp, vis_map_temp_list);
                //vis_map_temp=bf_pt(map_or_perc, crit_point, sens_size, vis_map_temp, false, false);
                vis_map_temp=bf_pt_v2(map_or_perc, crit_point, sens_size, vis_map_temp, false, false, vis_map_temp_list);
                //vis_map_temp=bf_pt_v2(map_or_perc, crit_point, sens_size, vis_map_temp, false, false);

                for(int xx=0;xx<vis_map_temp.rows;xx++)
                {
                    for(int yy=0;yy<vis_map_temp.cols;yy++)
                    {
                        if( (vis_map_temp.at<uchar>(xx,yy)==255) )
                        {
                            vis_map.at<uchar>(xx,yy)=255;

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
