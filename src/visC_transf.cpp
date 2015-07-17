#include "visC_transf.hpp"


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "labelling.hpp"
#include "CritPoints.hpp"
#include "clustering.hpp"
#include "bugfollowing.hpp"
#include "obs_extremes.hpp"
#include "ray.hpp"
#include "color.hpp"

using namespace std;

static const std::string M_WINDOW = "Map";
static const std::string E_WINDOW = "Erosion";
static const std::string C_WINDOW = "Close";
static const std::string L_WINDOW = "Labelled";
static const std::string A_WINDOW = "Actuation";
static const std::string V_WINDOW = "Visibility";
static const std::string D_WINDOW = "Debug";
static const std::string G_WINDOW = "Ground_Truth";

void VisC_transf::update_config(map_transform::ParametersConfig config)
{
    changed=true;
    changed2=true;

    infl=config.infl;
    defl=config.defl;
    _debug=config.debug;
    gt=config.ground_truth;
    rxr=config.x;
    ryr=config.y;
    scale=config.scale;
}


VisC_transf::VisC_transf(ros::NodeHandle nh): Vis_transf(nh)
{
    nh_.param("infl", infl, 5);
    nh_.param("defl", defl, infl);

    if(_debug){
        cv::namedWindow(M_WINDOW);
        cv::namedWindow(E_WINDOW);
        cv::namedWindow(C_WINDOW);
        if(pos_rcv)
        {
            cv::namedWindow(L_WINDOW);
            cv::namedWindow(A_WINDOW);
            cv::namedWindow(D_WINDOW);
            cv::namedWindow(V_WINDOW);
            if(gt && gt_c)
                cv::namedWindow(G_WINDOW);
        }
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

    }
}


void VisC_transf::show(void)
{
    if(count>0 && _debug){
        cv::imshow(M_WINDOW,map_or);
        cv::imshow(E_WINDOW,map_erosionOpPrintColor);
        cv::imshow(C_WINDOW,map_closeOp);
        if(pos_rcv)
        {
            cv::imshow(L_WINDOW,map_label);
            cv::imshow(A_WINDOW,map_act);
            cv::imshow(V_WINDOW,map_vis);
            cv::imshow(D_WINDOW,map_debug);
            if(gt && gt_c)
            {
                cv::imshow(G_WINDOW,map_truth);
            }
            else
            {
                cv::destroyWindow(G_WINDOW);
            }
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
    }
}

void VisC_transf::conf_space(void)
{
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                           cv::Size( 2*infl + 1, 2*infl+1 ),
                                           cv::Point( infl, infl ) );

    cv::Mat element_d = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                           cv::Size( 2*defl + 1, 2*defl+1 ),
                                           cv::Point( defl, defl ) );

    cv::Mat or_map, er_map, cr_map;

    or_map=cv_map.clone();
    msg_rcv_pub=msg_rcv;

    cv::erode( or_map, er_map, element);
    cv::dilate( er_map, cr_map, element_d);

    map_or=or_map;
    map_erosionOp=er_map;
    map_closeOp=cr_map;
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

vector<cv::Point> VisC_transf::expVisibility_obs(cv::Point2i crit, int defl, cv::Mat regions, uchar k, vector<float> extremes, unsigned obt_angle, cv::Mat &vis_map_temp)
{
    vector<cv::Point> occ;
    for(int rowx=max((crit.x-defl),0);rowx<=min((crit.x+defl),regions.rows-1);rowx++)
    {
        for(int coly=max((crit.y-defl),0);coly<=min((crit.y+defl),regions.cols-1);coly++)
        {
            float angle=atan2(coly-crit.y,rowx-crit.x);
            float dist=(rowx-crit.x)*(rowx-crit.x)+(coly-crit.y)*(coly-crit.y);

            bool reg;
            if(obt_angle==1)
                reg=(angle<extremes[1] && angle>extremes[0]);
            else
                reg=(angle<extremes[0] || angle>extremes[1]);

            if(reg && (dist<=(1*defl*defl)) && (regions.at<uchar>(rowx,coly)==(k+2) ) )
            {
                vis_map_temp.at<uchar>(rowx,coly)=255;
            }
            else if (reg && (dist<=(1*defl*defl)) && (map_or.at<uchar>(rowx,coly)==0) )
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

vector<cv::Point> VisC_transf::getExtremeFromObstacles(vector<cv::Point> occ, cv::Point2i crit)
{
    vector<vector<cv::Point> > occ_clust=cluster_points(occ);

    vector<cv::Point> occ_critP;

    cv::Mat contours = cv::Mat::ones(map_or.rows, map_or.cols, CV_8UC1)*255;


    for(unsigned int ind=0;ind<occ_clust.size();ind++)
    {
        for(unsigned int occ_p=0;occ_p<occ_clust[ind].size();occ_p++)
        {
            contours.at<uchar>(occ_clust[ind][occ_p].x,occ_clust[ind][occ_p].y)=0;
        }

        if(occ_clust[ind].size()==1)
        {
            occ_critP.push_back(cv::Point(occ_clust[ind][0].x,occ_clust[ind][0].y));
        }
        else
        {
            cv::Mat contours_check=contours.clone();
            bool stop=true;

            while(stop)
            {
                stop=false;
                cv::Point pos;
                for(unsigned int occ_p=0;occ_p<occ_clust[ind].size();occ_p++)
                {
                    if( contours_check.at<uchar>(occ_clust[ind][occ_p].x,occ_clust[ind][occ_p].y)==0  )
                    {
                        pos.x=occ_clust[ind][occ_p].x;
                        pos.y=occ_clust[ind][occ_p].y;
                        stop=true;
                        break;
                    }
                }

                if(stop)
                {
                    BugFollowing bf(contours, contours_check, pos);
                    vector<Chain> chain=bf.getChain();
                    contours_check=bf.getContourChecked();

                    ExtremesObst2Point eo(crit, chain);
                    vector<cv::Point> temp=eo.getExt();
                    occ_critP.insert(occ_critP.end(), temp.begin(), temp.end());
                }
            }

            if(occ_critP.size()==0)
            {
                occ_critP.push_back(cv::Point(occ_clust[ind][0].x,occ_clust[ind][0].y));
            }
        }
    }

    vector<cv::Point> occ_crit_filt;

    cv::Mat contours_filt=contours.clone();

    occ_crit_filt.clear();

    for(unsigned int c=0;c<occ_critP.size();c++)
    {
        if(contours_filt.at<uchar>(occ_critP[c].x,occ_critP[c].y)==255)
            continue;
        else
        {
            contours_filt.at<uchar>(occ_critP[c].x,occ_critP[c].y)=255;
            occ_crit_filt.push_back(cv::Point(occ_critP[c].x,occ_critP[c].y));
        }
    }

    return occ_crit_filt;
}

bool VisC_transf::valid_pos(cv::Point3i pos)
{
   return (map_erosionOp.at<uchar>(pos.x,pos.y)!=0);
}

void VisC_transf::visibility(cv::Point3i pos, bool proc, ros::Time t01)
{
    cv::Mat r_map=map_erosionOp.clone();

    bool new_v=reachability_map(pos,r_map);

    if( (new_v) || (prev.x<0) || (prev.y<0) || proc )  //if visibility is changed
    {
        int rad=min(infl,defl);  //if defl<infl, visibility is given by morphological closing


        cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                               cv::Size( 2*rad + 1, 2*rad+1 ),
                                               cv::Point( rad, rad ) );

        cv::Mat act_map=r_map.clone();

        dilate( act_map, act_map, element);  //actuation space

        cv::Mat vis_map=act_map.clone();

        Unreachable unreach(map_or, act_map);

        if(infl<defl)  //extended sensing radius
        {
            vis_map=ext_vis(unreach, vis_map, r_map);
        }

        map_label=r_map;
        map_act=act_map;
        map_vis=vis_map;

        map_debug=unreach.unreach_map;

        unsigned char color[3]={0,255,0};

        map_erosionOpPrintColor=printPoint(map_erosionOp, pos, color);

        ros::Duration diff = ros::Time::now() - t01;

        ROS_INFO("%s - Time for visibility: %f", tf_pref.c_str(), diff.toSec());

        if(gt)
        {
            ros::Time t3=ros::Time::now();

            map_truth=brute_force_opt_act(map_or, map_label, map_act ,defl);

            diff = ros::Time::now() - t3;

            ROS_INFO("%s - Time for  Optimized brute force: %f", tf_pref.c_str(), diff.toSec());

            gt_c=true;
        }
    }
}

cv::Mat VisC_transf::ext_vis(Unreachable unreach, cv::Mat vis_map, cv::Mat r_map)
{
    unreach.getFrontiers();

    cv::Mat regions=unreach.regions;

    cv::Mat vis_map_temp;

    CritPoints critP(map_or, r_map, infl);

    for (unsigned int k=0;k<unreach.frontiers.size();k++){//2;k++){//
        for(unsigned int ff=0;ff<unreach.frontiers[k].size();ff++)
        {
            vector<cv::Point> frontier=unreach.frontiers[k][ff];

            if(frontier.size()>0)
            {
                cv::Point2i crit=critP.find_crit_point(frontier);

                critP.frontier_extremes();

                //// TODO:neighbor points

                vis_map_temp = cv::Mat::zeros(regions.rows, regions.cols, CV_8UC1)*255;

                vector<cv::Point> occ=expVisibility_obs(crit, defl, regions, k, critP.getExtremes(), critP.getObt(), vis_map_temp);

                vector<cv::Point> occ_crit_filt=getExtremeFromObstacles(occ, crit);

                for(unsigned int c=0;c<occ_crit_filt.size();c++)
                {
                    raytracing(&vis_map_temp, cv::Point2i(crit.x,crit.y), occ_crit_filt[c], occ_crit_filt[c], defl);
                }

                for(unsigned int j=0;j<frontier.size();j++){
                    if(vis_map_temp.at<uchar>( frontier[j].x,frontier[j].y)==255)
                    {
                        std::vector<cv::Point> points_vis=label_seed(vis_map_temp.clone()/255,4,cv::Point(frontier[j].x,frontier[j].y));
                        for(unsigned int pv=0;pv<points_vis.size();pv++)
                        {
                            vis_map.at<uchar>(points_vis[pv].x,points_vis[pv].y)=255;
                        }
                        break;
                    }
                }
            }
        }
    }

    return vis_map;
}
