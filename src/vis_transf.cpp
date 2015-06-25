#include "vis_transf.hpp"

Vis_transf::Vis_transf(ros::NodeHandle nh): nh_(nh)
{
    pub = nh_.advertise<nav_msgs::OccupancyGrid>("e_map", 1,true);
    pub2 = nh_.advertise<nav_msgs::OccupancyGrid>("c_map", 1,true);
    pub3 = nh_.advertise<nav_msgs::OccupancyGrid>("r_map", 1,true);
    pub4 = nh_.advertise<nav_msgs::OccupancyGrid>("a_map", 1,true);
    pub5 = nh_.advertise<nav_msgs::OccupancyGrid>("v_map", 1,true);
    pub6 = nh_.advertise<nav_msgs::OccupancyGrid>("g_map", 1,true);
    sub = nh_.subscribe("map", 1, &Vis_transf::rcv_map, this);
    nh_.param("infl", infl, 5);
    nh_.param("defl", defl, infl);
    nh_.param("x", rxr, 50.0);
    nh_.param("y", ryr, 50.0);
    nh_.param("scale", scale, 100.0);
    scale/=100;

    nh_.param("tf_prefix", tf_pref, std::string(""));
    nh_.param("ground_truth", gt, false);
    nh_.param("debug", _debug, true);
    prev.x=-1; prev.y=-1;
    count=0;
    func = boost::bind(&Vis_transf::callbackParameters, this,_1, _2);
    server.setCallback(func);
    treated=false;
    treated2=true;
    pos_rcv=false;
    gt_c=false;
    changed=false;
    changed2=false;
    changed_p=false;

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

Vis_transf::~Vis_transf()
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

void Vis_transf::callbackParameters(map_transform::ParametersConfig &config, uint32_t level) {
    mtx.lock();
    changed_p=true;
    _config=config;
    mtx.unlock();
}


void Vis_transf::show(void)
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


nav_msgs::OccupancyGrid Vis_transf::Mat2RosMsg(cv::Mat map ,const nav_msgs::OccupancyGrid& msg)
{
    //map=scaling(map, 1/scale);

    nav_msgs::OccupancyGrid n_msg;
    n_msg.header.stamp = ros::Time::now();
    n_msg.header.frame_id = msg.header.frame_id;
    n_msg.info=msg.info;

    n_msg.data.resize(msg.data.size(),-1);
    std::vector<signed char>::const_iterator mapDataIter = msg.data.begin();
    std::vector<signed char>::iterator n_mapDataIter = n_msg.data.begin();
    for(unsigned int i=0;i<msg.info.height;i++){
        for(unsigned int j=0;j<msg.info.width;j++){
                signed char val;
                if( *mapDataIter < 0)
                    val = -1;
                else if(map.at<unsigned char>(j,i) == 255)
                    val = 0;
                else
                    val = 100;

                *n_mapDataIter=val;

                n_mapDataIter++;
                mapDataIter++;
            }
    }
    return n_msg;
}



void Vis_transf::rcv_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("I heard map: [%d]", msg->header.seq);

    cv::Mat prev_map=cv_map.clone();

    cv_map = cv::Mat(msg->info.width, msg->info.height, CV_8UC1);

    treated=true;

    res= msg->info.resolution;
    height= msg->info.height;
    width= msg->info.width;

    or_x= msg->info.origin.position.x;
    or_y= msg->info.origin.position.y;

    if(count>0)
    {
        if(cv_map.rows!=prev_map.rows || cv_map.cols!=prev_map.cols || res!=msg->info.resolution || or_x!=msg->info.origin.position.x || or_y!=msg->info.origin.position.y )
            treated=false;
    }
    else
        treated=false;


    std::vector<signed char>::const_iterator mapDataIterC = msg->data.begin();
    signed char map_occ_thres = 90;
    for(unsigned int i=0;i<msg->info.height;i++){
        for(unsigned int j=0;j<msg->info.width;j++){
            unsigned char val_cv;
            if(*mapDataIterC >  map_occ_thres)
            {val_cv=0;}
            else if(*mapDataIterC == 0)
            {val_cv=255;}
            else
            {val_cv=255;}
            cv_map.at<uchar>(j,i) = val_cv;

            if(count>0 && treated)
                if(val_cv!=prev_map.at<uchar>(j,i))
                    treated=false;

            mapDataIterC++;
        }
    }

    ++count;

    msg_rcv=*msg;

    //cv_map=scaling(cv_map, scale);

    //res=res/scale;
    //height= cv_map.cols;
    //width= cv_map.rows;

    //or_x= msg->info.origin.position.x;
    //or_y= msg->info.origin.position.y;

    treated2=treated;
}

bool Vis_transf::checkProceed2(void)
{
    bool proc=false;

    if((!treated2) || changed2)
    {
        proc=true;
        treated2=true;
        changed2=false;
    }

    return proc;
}

bool Vis_transf::checkProceed(void)
{
    bool proc=false;

    if((!treated) || changed)
    {
        proc=true;
        treated=true;
        changed=false;
    }

    return proc;
}

void Vis_transf::transf(void)
{
    bool proc=checkProceed2();

    if(count>0 && (proc) )
    {
        ros::Time t01=ros::Time::now();

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

        ros::Duration diff = ros::Time::now() - t01;

        ROS_INFO("%s - Time for configuration space: %f", tf_pref.c_str(), diff.toSec());
    }
}

cv::Mat Vis_transf::printPoint(cv::Mat img, cv::Point2i pos, unsigned char* color)
{
    vector<cv::Mat> channels(3);

    channels[0]=img.clone();
    channels[1]=img.clone();
    channels[2]=img.clone();

    for(int k=0;k<3;k++)
    {
        for(int i=(pos.x-1);i<=(pos.x+1);i++)
        {
            for(int j=(pos.y-1);j<=(pos.y+1);j++)
            {
                channels[k].at<uchar>(boundPos(i,img.rows),boundPos(j,img.cols))=color[2-k];
            }
        }
    }

    cv::Mat ret;

    cv::merge(channels, ret);

    return ret;
}

bool Vis_transf::getTFPosition(cv::Point2d &p)
{
    tf::StampedTransform transform;
    try{
        pos_listener.lookupTransform("/map", tf_pref+"/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_INFO("%s",ex.what());
      return false;
    }
    p.x=transform.getOrigin().x();
    p.y=transform.getOrigin().y();
    return true;
}

bool Vis_transf::getPosition(cv::Point2i& pos){
    cv::Point2d p;
    if(!_debug)
    {
        if(!getTFPosition(p))
            return false;
    }
    else
    {
        p.x=rxr/100.0*map_or.rows*res;
        p.y=ryr/100.0*map_or.cols*res;
    }

    pos.x=(int) round((p.x-or_x)/res);
    pos.y=(int) round((p.y-or_y)/res);

    pos.x=boundPos(pos.x, map_or.rows);
    pos.y=boundPos(pos.y, map_or.cols);

    return true;
}

bool Vis_transf::reachability_map(std::vector<std::vector<cv::Point> > labels, cv::Point2i pos, cv::Mat & r_map)
{
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

vector<cv::Point> Vis_transf::expVisibility_obs(cv::Point2i crit, int defl, cv::Mat regions, uchar k, vector<float> extremes, unsigned obt_angle, cv::Mat &vis_map_temp)
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

vector<cv::Point> Vis_transf::getExtremeFromObstacles(vector<cv::Point> occ, cv::Point2i crit)
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


void Vis_transf::transf_pos(void)
{
    if(count>0)
    {
        ros::Time t01=ros::Time::now();

        cv::Point2i pos;

        if (!getPosition(pos))
            return;

        bool proc=checkProceed();

        if(map_erosionOp.at<uchar>(pos.x,pos.y)==0)  //invalid center position of the robot (touching obstacles or walls)
        {
            gt_c=false;

            pos_rcv=true;

            unsigned char color[3]={255,0,0};

            map_erosionOpPrintColor=printPoint(map_erosionOp, pos, color);

            map_label=cv::Mat::zeros(map_erosionOp.rows, map_erosionOp.cols, CV_8UC1);
            map_act=map_label;
            map_vis=map_label;
            map_debug=map_label;

            bool print=true;

            if (prev.x>=0 && prev.y>=0 && prev.x<map_erosionOp.rows && prev.y<map_erosionOp.cols)
                if (map_erosionOp.at<uchar>(prev.x,prev.y)==0 && !proc)
                {
                    print=false;
                }

            prev=pos;

            if(print)
            {
                ros::Duration diff = ros::Time::now() - t01;
                ROS_INFO("%s - Time for visibility (invalid position): %f", tf_pref.c_str(), diff.toSec());
            }

            return;
        }


        std::vector<std::vector<cv::Point> > labels=label(map_erosionOp.clone()/255,8);

        cv::Mat r_map=map_erosionOp.clone();

        bool new_v=reachability_map(labels,pos,r_map);

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

        pos_rcv=true;

        prev=pos;
    }
    else
        return;
}

void Vis_transf::update(void)
{
    bool proc=false;

    map_transform::ParametersConfig config;

    mtx.lock();

    if(changed_p)
    {
        changed_p=false;
        config=_config;
        proc=true;
    }

    mtx.unlock();

    if(proc)
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
}

void Vis_transf::publish(void)
{
    if(count>0)
    {
        nav_msgs::OccupancyGrid n_msg;

        n_msg=Mat2RosMsg(map_erosionOp , msg_rcv_pub);
        pub.publish(n_msg);

        n_msg=Mat2RosMsg(map_closeOp , msg_rcv_pub);
        pub2.publish(n_msg);

        if(pos_rcv)
        {
            n_msg=Mat2RosMsg( map_label , msg_rcv_pub);
            pub3.publish(n_msg);

            n_msg=Mat2RosMsg( map_act , msg_rcv_pub);
            pub4.publish(n_msg);

            n_msg=Mat2RosMsg( map_vis , msg_rcv_pub);
            pub5.publish(n_msg);

            if(gt  && gt_c)
            {
                n_msg=Mat2RosMsg( map_truth , msg_rcv_pub);
                pub6.publish(n_msg);
            }
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "visibility", ros::init_options::AnonymousName);

    ros::NodeHandle nh("~");

    Vis_transf vis(nh);

    ros::Rate loop_rate(10);


    while (ros::ok())
    {
        vis.update();

        ros::spinOnce();

        vis.transf();

        vis.transf_pos();

        vis.show();

        vis.publish();

        loop_rate.sleep();
    }


    return 0;
}
