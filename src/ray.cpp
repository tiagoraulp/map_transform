#include "ray.hpp"
#include <cmath>

using namespace std;

bool raytracing(cv::Mat *map, cv::Point2i opt, cv::Point2i ref, cv::Point2i dest, float dist_t, bool (*func)(cv::Mat *,cv::Point2i,cv::Point2i),  cv::Mat & test_pt, vector<cv::Point> & list)
{
    bool boost=true, boost_list=true;
    if(test_pt.rows!=map->rows && test_pt.cols!=map->cols)
        boost=false;
    if(list.size()!=1)
        boost_list=false;

    float angle=atan2(dest.y-opt.y, dest.x-opt.x);

    float cos_ang=cos(angle);
    float sin_ang=sin(angle);

    float sign_x=0; if(cos_ang!=0) sign_x=cos_ang/abs(cos_ang);
    float sign_y=0; if(sin_ang!=0) sign_y=sin_ang/abs(sin_ang);

    int p_x=ref.x;
    int p_y=ref.y;

    float temp, tempx=ref.x, tempy=ref.y, temp_tx, temp_ty;

    if(cos_ang!=0)
    {
        temp_tx=sign_x*0.5/cos_ang;
        if(sin_ang!=0)
        {
            temp_ty=sign_y*0.5/sin_ang;
            if(temp_tx<temp_ty)
                temp=temp_tx;
            else
                temp=temp_ty;
        }
        else
            temp=temp_tx;
    }
    else
        temp=sign_y*0.5/sin_ang;

    int prev_px=p_x;
    int prev_py=p_y;

    if(sign_x==0)
        p_x=p_x;
    else
        p_x=sign_x*((int)round(sign_x*(tempx+temp*cos_ang)+p_x)-p_x);

    if(sign_y==0)
        p_y=p_y;
    else
        p_y=sign_y*((int)round(sign_y*(tempy+temp*sin_ang)+p_y)-p_y);

    float dist=sqrt( (ref.x-opt.x)*(ref.x-opt.x)+(ref.y-opt.y)*(ref.y-opt.y) );

    while( (dist+temp)<=dist_t && p_x>=0 && p_x<map->rows && p_y>=0 && p_y<map->cols )
    {
        dist=dist+temp;

        if(!(*func)(map, cv::Point2i(p_x,p_y), cv::Point2i(prev_px,prev_py)))
            return false;

        if(boost)
            test_pt.at<uchar>(p_x,p_y)=0;

        if(boost_list)
            list.push_back(cv::Point(p_x,p_y));

        tempx=tempx+temp*cos_ang;
        tempy=tempy+temp*sin_ang;

        if(cos_ang!=0)
        {
            temp_tx=(p_x+sign_x*0.5-tempx)/cos_ang;
            if(temp_tx==0)
                temp_tx=sign_x*1/cos_ang;
            if(sin_ang!=0)
            {
                temp_ty=(p_y+sign_y*0.5-tempy)/sin_ang;
                if(temp_ty==0)
                    temp_ty=sign_y*1/sin_ang;

                if(temp_tx<temp_ty)
                    temp=temp_tx;
                else
                    temp=temp_ty;
            }
            else
                temp=temp_tx;
        }
        else
        {
            temp=(p_y+sign_y*0.5-tempy)/sin_ang;
            if(temp==0)
                temp=sign_y*1/sin_ang;
        }

        prev_px=p_x;
        prev_py=p_y;

        if(sign_x==0)
            p_x=p_x;
        else
            p_x=sign_x*((int)round(sign_x*(tempx+temp*cos_ang)+p_x)-p_x);

        if(sign_y==0)
            p_y=p_y;
        else
            p_y=sign_y*((int)round(sign_y*(tempy+temp*sin_ang)+p_y)-p_y);
    }
    return true;
}


bool checkMap(cv::Mat *map, cv::Point2i p, cv::Point2i prev)
{
    if( map->at<uchar>(p.x,p.y)==0 )
        return false;

    if( (abs(prev.x-p.x)+abs(prev.y-p.y))==2 )
    {
        if( map->at<uchar>(prev.x,p.y)==0 )
            return false;

        if( map->at<uchar>(p.x,prev.y)==0 )
            return false;
    }

    return true;
}


bool print2Map(cv::Mat *map, cv::Point2i p, cv::Point2i prev)//, cv::Point2i prev)
{
    map->at<uchar>(p.x,p.y)=0;
    return true;
}

bool raytracing(cv::Mat map, int opt_x, int opt_y, int dest_x, int dest_y, cv::Mat & test_pt, vector<cv::Point> & list)
{
    float dist_t=sqrt( (dest_x-opt_x)*(dest_x-opt_x)+(dest_y-opt_y)*(dest_y-opt_y) );
    return raytracing(&map, cv::Point2i(opt_x,opt_y), cv::Point2i(opt_x,opt_y), cv::Point2i(dest_x,dest_y), dist_t, &checkMap, test_pt, list);
}

bool raytracing(cv::Mat map, int opt_x, int opt_y, int dest_x, int dest_y, bool t, cv::Mat & test_pt, vector<cv::Point> & list)
{
    float dist_t=sqrt( (dest_x-opt_x)*(dest_x-opt_x)+(dest_y-opt_y)*(dest_y-opt_y) );
    if(opt_x<map.rows && opt_x>=0 && opt_y<map.cols && opt_y>=0 && dest_x<map.rows && dest_x>=0 && dest_y<map.cols && dest_y>=0)
        return raytracing(&map, cv::Point2i(opt_x,opt_y), cv::Point2i(opt_x,opt_y), cv::Point2i(dest_x,dest_y), dist_t, &checkMap, test_pt, list);
    else
        return false;
}

bool raytracing(cv::Mat map, int opt_x, int opt_y, int dest_x, int dest_y, bool t)
{
    cv::Mat temp=cv::Mat(0,0,CV_8UC1);
    vector<cv::Point> list;
    return raytracing(map, opt_x, opt_y, dest_x, dest_y, temp, list);
}

void raytracing(cv::Mat* map, cv::Point2i opt, cv::Point2i ref, cv::Point2i dest, float dist_t)
{
    cv::Mat temp=cv::Mat(0,0,CV_8UC1);
    vector<cv::Point> list;
    raytracing(map, opt, ref, dest, dist_t, &print2Map, temp, list);
}


bool bf_sq(cv::Mat map, cv::Mat reach, int defl, int i, int j, cv::Mat & test_pt, vector<cv::Point> & list)
{
    bool stop=false;

    for(int ii=0;ii<reach.rows;ii++)
    {
        for(int jj=0;jj<reach.cols;jj++)
        {
            if( ( (i-ii)*(i-ii)+(j-jj)*(j-jj) )>defl*defl )
                continue;
            if(reach.at<uchar>(ii,jj)==0)
                continue;
            if( raytracing(map,ii,jj,i,j,test_pt,list) )
            {
                stop=true;
                break;
            }
        }
        if(stop)
            break;
    }

    return stop;
}

bool bf_sq(cv::Mat map, vector<cv::Mat> reach, Elem sensor, int i, int j, cv::Mat & test_pt, vector<cv::Point> & list)
{
    bool stop=false;

    for(unsigned int a=0; a<reach.size();a++)
    {
        for(int ii=0;ii<reach[a].rows;ii++)
        {
            for(int jj=0;jj<reach[a].cols;jj++)
            {
                if(reach[a].at<uchar>(ii,jj)==0)
                    continue;

                if( (i+sensor.pu-ii+sensor.pt.x)<sensor.elems[a].rows &&  (i+sensor.pu-ii+sensor.pt.x)>=0 && (j+sensor.pl-jj+sensor.pt.y)<sensor.elems[a].cols &&  (j+sensor.pl-jj+sensor.pt.y)>=0 )
                {
                    if(sensor.elems[a].at<uchar>(i+sensor.pu-ii+sensor.pt.x, j+sensor.pl-jj+sensor.pt.y)==0)
                    {
                        continue;
                    }
                    else
                    {
                        if( raytracing(map,ii-sensor.pu+sensor.pt2[a].x-sensor.pt.x,jj-sensor.pl+sensor.pt2[a].y-sensor.pt.y,i,j, true,test_pt, list) )
                        {
                            stop=true;
                            break;
                        }
                    }
                }
                else
                {
                    continue;
                }
            }
            if(stop)
                break;
        }
        if(stop)
            break;
    }

    return stop;
}

bool bfo(cv::Mat map, cv::Mat reach, int defl, int i, int j, vector<cv::Point> hlx, cv::Mat & test_pt, vector<cv::Point> & list)
{
    for(unsigned int r=0; r<hlx.size(); r++)
    {
        int ii=i+hlx[r].x, jj=j+hlx[r].y;

        if( ( (i-ii)*(i-ii)+(j-jj)*(j-jj) )>defl*defl )
        {
            continue;
        }
        if( (ii)<reach.rows &&  (ii)>=0 && (jj)<reach.cols &&  (jj)>=0 )
        {
            if(reach.at<uchar>(ii,jj)==0)
            {
                continue;
            }
            else
            {
                if( raytracing(map,ii,jj,i,j,test_pt, list) )
                {
                    return true;
                }
            }
        }
    }

    return false;
}

bool bfo(cv::Mat map, vector<cv::Mat> reach, Elem sensor, int i, int j, vector<cv::Point> hlx, cv::Mat & test_pt, vector<cv::Point> & list)
{
    for(unsigned int r=0; r<hlx.size(); r++)
    {
        int ii=i+hlx[r].x, jj=j+hlx[r].y;

        for(unsigned int a=0; a<reach.size(); a++)
        {
            if( (ii+sensor.pu)<reach[a].rows &&  (ii+sensor.pu)>=0 && (jj+sensor.pl)<reach[a].cols &&  (jj+sensor.pl)>=0 )
            {
                if(reach[a].at<uchar>(ii+sensor.pu,jj+sensor.pl)==0)
                {
                    continue;
                }
                else
                {
                    if( (i-ii+sensor.pt.x)<sensor.elems[a].rows &&  (i-ii+sensor.pt.x)>=0 && (j-jj+sensor.pt.y)<sensor.elems[a].cols &&  (j-jj+sensor.pt.y)>=0 )
                    {
                        if(sensor.elems[a].at<uchar>(i-ii+sensor.pt.x, j-jj+sensor.pt.y)==0)
                        {
                            continue;
                        }
                        else
                        {
                            if( raytracing(map,ii+sensor.pt2[a].x-sensor.pt.x,jj+sensor.pt2[a].y-sensor.pt.y,i,j, true,test_pt, list) )
                            {
                                return true;
                            }
                        }
                    }
                }
            }
            else
            {
                break;
            }
        }
    }

    return false;
}

bool bf_sq(cv::Mat map, vector<cv::Point> reach, int defl, int i, int j, cv::Mat & test_pt, vector<cv::Point> & list)
{
    for(unsigned int r=0; r<reach.size(); r++)
    {
        int ii=reach[r].x;
        int jj=reach[r].y;

        if( ( (i-ii)*(i-ii)+(j-jj)*(j-jj) )>defl*defl )
            continue;
        else
        {
            if( raytracing(map,ii,jj,i,j,test_pt, list) )
            {
                return true;
            }
        }
    }

    return false;
}

bool bfo(cv::Mat map, vector<cv::Point> reach, int defl, int i, int j, vector<cv::Point> hlx, cv::Mat & test_pt, vector<cv::Point> & list)
{
    return bf_sq(map, reach, defl, i, j, test_pt, list);
}


bool bf_sq(cv::Mat map, vector<cv::Point3i> reach, Elem sensor, int i, int j, cv::Mat & test_pt, vector<cv::Point> & list)
{
    for(unsigned int r=0; r<reach.size(); r++)
    {
        int ii=reach[r].x;
        int jj=reach[r].y;
        int a=reach[r].z;

        if( (i+sensor.pu-ii+sensor.pt.x)<sensor.elems[a].rows &&  (i+sensor.pu-ii+sensor.pt.x)>=0 && (j+sensor.pl-jj+sensor.pt.y)<sensor.elems[a].cols &&  (j+sensor.pl-jj+sensor.pt.y)>=0 )
        {
            if(sensor.elems[a].at<uchar>(i+sensor.pu-ii+sensor.pt.x, j+sensor.pl-jj+sensor.pt.y)==0)
            {
                continue;
            }
            else
            {
                if( raytracing(map,ii-sensor.pu+sensor.pt2[a].x-sensor.pt.x,jj-sensor.pl+sensor.pt2[a].y-sensor.pt.y,i,j, true,test_pt, list) )
                {
                    return true;
                }
            }
        }
        else
        {
            continue;
        }

    }
    return false;
}

bool bfo(cv::Mat map, vector<cv::Point3i> reach, Elem sensor, int i, int j, vector<cv::Point> hlx, cv::Mat & test_pt, vector<cv::Point> & list)
{
    return bf_sq(map, reach, sensor, i, j, test_pt, list);
}

vector<cv::Point> bf_hlx(int defl)
{
    vector<cv::Point> hlx;
    for(int r=0; r<=defl; r++)
    {
        if(r==0)
        {
            hlx.push_back(cv::Point(0,0));
        }
        else
        {
            for(int p=-r;p<r;p++)
            {
                int ii=0-r, jj=0+p;
                hlx.push_back(cv::Point(ii,jj));

                ii=0+p, jj=0+r;
                hlx.push_back(cv::Point(ii,jj));

                ii=0+r, jj=0-p;
                hlx.push_back(cv::Point(ii,jj));

                ii=0-p, jj=0-r;
                hlx.push_back(cv::Point(ii,jj));
            }
        }
    }
    return hlx;
}


vector<cv::Point> bf_hlx(Elem sensor)
{
    return bf_hlx(sensor.pb);
}

void vector_reserve(vector<cv::Point> & list, int defl)
{
    list.reserve(2*defl);
}

void vector_reserve(vector<cv::Point> & list, Elem sensor)
{
    vector_reserve(list, 2*sensor.pb);
}

template <typename T, typename T2>
cv::Mat brute_force(cv::Mat map, T reach, T2 defl, bool opt, cv::Mat act, bool opt_rep, bool opt_repM)
{
    cv::Mat result=map.clone();

    cv::Mat test_points=cv::Mat::ones(map.rows, map.cols, CV_8UC1)*255, test_pt_temp=cv::Mat(0,0,CV_8UC1);//, test_pt_temp2=cv::Mat::ones(map.rows, map.cols, CV_8UC1)*255;

    if(opt_repM)
        opt_rep=false;

    vector<cv::Point> pt_vis_list;

    bool act_test;

    if(act.rows!=map.rows || act.cols!=map.cols)
        act_test=false;
    else
        act_test=true;

    vector<cv::Point> hlx;
    if(opt)
    {
       hlx=bf_hlx(defl);
    }

    for(int i=0;i<map.rows;i++)//152;i++)//
    {
        for(int j=0;j<map.cols;j++)//117;j++)//
        {
            //if(j<50 || i<60)
            //    ;
            //else
            //    continue;
            //cout<<i<<" "<<j<<endl;

            if(map.at<uchar>(i,j)==0)
                continue;

            if(act_test)
            {
                if(act.at<uchar>(i,j)==255)
                    continue;
            }

            if(opt_rep || opt_repM)
            {
                if(test_points.at<uchar>(i,j)!=255)
                {
                    //test_pt_temp2.at<uchar>(i,j)=0;
                    continue;
                }
            }

            if(opt_repM)
            {
                pt_vis_list.assign(1, cv::Point(i,j));
            }

            if(opt_rep)
            {
                test_pt_temp=test_points.clone();
            }


            bool found=false;

            if(opt)
            {
                if(bfo( map, reach, defl, i, j, hlx, test_pt_temp , pt_vis_list) )
                    found=true;
            }
            else
            {
                if(bf_sq( map, reach, defl, i, j, test_pt_temp, pt_vis_list) )
                    found=true;
            }

            if(found)
            {
                if(opt_rep)
                {
                    test_points=test_pt_temp.clone();
                }

                if(opt_repM)
                {
                    for(unsigned int i=0;i<pt_vis_list.size();i++)
                    {
                        test_points.at<uchar>(pt_vis_list[i].x,pt_vis_list[i].y)=0;
                    }
                }

                continue;
            }

            result.at<uchar>(i,j)=0;
        }
    }

    return result;
}

template <typename T2>
cv::Mat bf_pt(cv::Mat map, cv::Point pt, T2 defl, cv::Mat vis, bool opt_rep, bool opt_repM)
{
    cv::Mat result=vis.clone();

    cv::Mat test_points=cv::Mat::ones(map.rows, map.cols, CV_8UC1)*255, test_pt_temp=cv::Mat(0,0,CV_8UC1);

    if(opt_repM)
        opt_rep=false;

    vector<cv::Point> pt_vis_list;

    vector<cv::Point> reach;
    reach.clear();
    reach.push_back(pt);

    for(int i=0;i<vis.rows;i++)//152;i++)//
    {
        for(int j=0;j<vis.cols;j++)//117;j++)//
        {
            //if(j<50 || i<60)
            //    ;
            //else
            //    continue;
            //cout<<i<<" "<<j<<endl;

            if(vis.at<uchar>(i,j)==0)
                continue;

            if(opt_rep || opt_repM)
            {
                if(test_points.at<uchar>(i,j)!=255)
                {
                    continue;
                }
            }

            if(opt_repM)
            {
                pt_vis_list.assign(1, cv::Point(i,j));
            }

            if(opt_rep)
            {
                test_pt_temp=test_points.clone();
            }

            bool found=false;

            if(bf_sq( map, reach , defl, i, j, test_pt_temp, pt_vis_list) )
                found=true;

            if(found)
            {
                if(opt_rep)
                {
                    test_points=test_pt_temp.clone();
                }

                if(opt_repM)
                {
                    for(unsigned int i=0;i<pt_vis_list.size();i++)
                    {
                        test_points.at<uchar>(pt_vis_list[i].x,pt_vis_list[i].y)=0;
                    }
                }

                continue;
            }

            result.at<uchar>(i,j)=0;
        }
    }

    return result;
}

float coverage(cv::Mat test, cv::Mat truth)
{
    if(test.size!=truth.size)
        return 0;

    long int sum_t=0, sum=0;

    for(signed int i=0; i<truth.rows; i++)
    {
        for(signed int j=0; j<truth.cols; j++)
        {
            if(truth.at<uchar>(i,j)==255)
            {
                sum_t++;
                if(test.at<uchar>(i,j)==255)
                {
                    sum++;
                }
            }
        }
    }

    return (((float)sum)/((float)sum_t));
}

vector<int> confusion_matrix(cv::Mat test, cv::Mat truth)
{
    vector<int> result;
    result.empty();
    if(test.size!=truth.size)
        return result;

    result.assign(4,0);

    long int sum_t=0, sumTP=0, sumTN=0, sumFP=0, sumFN=0;

    for(signed int i=0; i<truth.rows; i++)
    {
        for(signed int j=0; j<truth.cols; j++)
        {
            sum_t++;

            if(truth.at<uchar>(i,j)==255)
            {
                if(test.at<uchar>(i,j)==255)
                {
                    sumTP++;
                }
                else
                {
                    sumFN++;
                }
            }
            else
            {
                if(test.at<uchar>(i,j)==255)
                {
                    sumFP++;
                }
                else
                {
                    sumTN++;
                }
            }
        }
    }

    result[0]=sumTP;
    result[1]=sumFP;
    result[2]=sumFN;
    result[3]=sumTN;

    return result;
}

template cv::Mat bf_pt<int>(cv::Mat map, cv::Point pt, int defl, cv::Mat vis, bool opt_rep, bool opt_repM);
template cv::Mat brute_force<cv::Mat, int>(cv::Mat map, cv::Mat reach, int defl, bool opt, cv::Mat act, bool opt_rep, bool opt_repM);
template cv::Mat brute_force<vector<cv::Mat>, Elem>(cv::Mat map, vector<cv::Mat> reach, Elem defl, bool opt, cv::Mat act, bool opt_rep, bool opt_repM);
template cv::Mat brute_force<vector<cv::Point>, int>(cv::Mat map, vector<cv::Point> reach, int defl, bool opt, cv::Mat act, bool opt_rep, bool opt_repM);
template cv::Mat brute_force<vector<cv::Point3i>, Elem>(cv::Mat map, vector<cv::Point3i> reach, Elem defl, bool opt, cv::Mat act, bool opt_rep, bool opt_repM);
