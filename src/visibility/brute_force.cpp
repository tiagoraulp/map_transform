#include "brute_force.hpp"
#include "ray.hpp"
#include "vector_utils.hpp"

#include <iostream>

using namespace std;

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
    if(i==182 && j==11)
        cout<<"New Test"<<endl;
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
                if(i==182 && j==11)
                    cout<<"From: "<<ii<<"; "<<jj<<endl;
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

/// opt_rep - while doing raycasting, saves valid points as visible - done with matrix
/// opt_repM - same as before, but with list instead of matrix - should be more efficient
///
/// opt - helix around test point for reach search, instead of square matrix

template <typename T, typename T2>
cv::Mat brute_force(cv::Mat map, T reach, T2 defl, bool opt, cv::Mat act, bool opt_rep, bool opt_repM)
{
    cv::Mat result=map.clone();

    cv::Mat test_points=cv::Mat::ones(map.rows, map.cols, CV_8UC1)*255, test_pt_temp=cv::Mat(0,0,CV_8UC1);

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

            if(i==182 && j==3)
                cout<<"Test182;3!!!!"<<endl;

            if(i==182 && j==11)
                cout<<"Test182;11!!!!"<<endl;


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

            if(i==182 && j==11)
                cout<<"Test182;11"<<endl;

            if(i==182 && j==3)
                cout<<"Test182;3"<<endl;


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

            /// TODO: check if lists can be used even if ray casting returns false
            /// in principle yes, because lists include valid points until raycasting detected obstacle and returned false
            /// but needs to guarantee false points are not included (e.g. last point)

            if(opt_rep)
            {
                test_points=test_pt_temp.clone();
            }

            if(opt_repM)
            {
                for(unsigned int ir=0;ir<pt_vis_list.size();ir++)
                {
                    if( found || (pt_vis_list[ir].x!=i || pt_vis_list[ir].y!=j) )
                        test_points.at<uchar>(pt_vis_list[ir].x,pt_vis_list[ir].y)=0;
                }
            }

            if(found)
            {
                continue;
            }

            result.at<uchar>(i,j)=0;
        }
    }

    return result;
}

template <typename T, typename T2>
cv::Mat bf_pt(cv::Mat map, T pt, T2 defl, cv::Mat vis, bool opt_rep, bool opt_repM)
{
    cv::Mat result=vis.clone();

    cv::Mat test_points=cv::Mat::ones(map.rows, map.cols, CV_8UC1)*255, test_pt_temp=cv::Mat(0,0,CV_8UC1);

    if(opt_repM)
        opt_rep=false;

    vector<cv::Point> pt_vis_list;

    vector<T> reach;
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

            if(i==182 && j==3)
                cout<<"Test182;3!!!!"<<endl;

            if(i==182 && j==11)
                cout<<"Test182;11!!!!"<<endl;


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

            if(i==182 && j==11)
                cout<<"Test182;11"<<endl;

            if(i==182 && j==3)
                cout<<"Test182;3"<<endl;


            if(bf_sq( map, reach , defl, i, j, test_pt_temp, pt_vis_list) )
                found=true;

            /// TODO: check if lists can be used even if ray casting returns false
            /// in principle yes, because lists include valid points until raycasting detected obstacle and returned false
            /// but needs to guarantee false points are not included (e.g. last point)

            if(opt_rep)
            {
                test_points=test_pt_temp.clone();
            }

            if(opt_repM)
            {
                for(unsigned int ir=0;ir<pt_vis_list.size();ir++)
                {
                    if( found || (pt_vis_list[ir].x!=i || pt_vis_list[ir].y!=j) )
                        test_points.at<uchar>(pt_vis_list[ir].x,pt_vis_list[ir].y)=0;
                    if(pt_vis_list[ir].x==182 && pt_vis_list[ir].y==11)
                        cout<<"182,11 by: "<< i<<"; "<<j <<endl;
                }
            }

            if(found)
            {
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

template cv::Mat bf_pt<cv::Point, int>(cv::Mat map, cv::Point pt, int defl, cv::Mat vis, bool opt_rep, bool opt_repM);
template cv::Mat bf_pt<cv::Point3i, Elem>(cv::Mat map, cv::Point3i pt, Elem defl, cv::Mat vis, bool opt_rep, bool opt_repM);
template cv::Mat brute_force<cv::Mat, int>(cv::Mat map, cv::Mat reach, int defl, bool opt, cv::Mat act, bool opt_rep, bool opt_repM);
template cv::Mat brute_force<vector<cv::Mat>, Elem>(cv::Mat map, vector<cv::Mat> reach, Elem defl, bool opt, cv::Mat act, bool opt_rep, bool opt_repM);
template cv::Mat brute_force<vector<cv::Point>, int>(cv::Mat map, vector<cv::Point> reach, int defl, bool opt, cv::Mat act, bool opt_rep, bool opt_repM);
template cv::Mat brute_force<vector<cv::Point3i>, Elem>(cv::Mat map, vector<cv::Point3i> reach, Elem defl, bool opt, cv::Mat act, bool opt_rep, bool opt_repM);
