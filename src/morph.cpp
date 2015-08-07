#include "morph.hpp"

#include <opencv2/imgproc/imgproc.hpp>

#include "vector_utils.hpp"

using namespace std;


Elem multiRobot(cv::Mat elem, cv::Point2f& pt, double orient ,double scale, int res, cv::Point2f& pt2)
{
    Elem result;

    int pl, pr, pu, pb; //add lines for elem to deal with scale and rotation

    pu=elem.rows/2*max(0.0,scale-1);
    pb=elem.rows/2*max(0.0,scale-1);
    pl=elem.cols/2*max(0.0,scale-1);
    pr=elem.cols/2*max(0.0,scale-1);

    cv::copyMakeBorder(elem,elem,pu,pb,pl,pr,cv::BORDER_CONSTANT,cv::Scalar(0));

    pt.x=pt.x+pu;
    pt.y=pt.y+pl;

    pt2.x=pt2.x+pu;
    pt2.y=pt2.y+pl;


    int max_r=max(pt.x,elem.rows-pt.x), max_c=max(pt.y,elem.cols-pt.y);

    int max_d=(int)(sqrt(max_r*max_r+max_c*max_c));

    pl=max_d-pt.y+1;
    pr=max_d-elem.cols+pt.y+1;
    pu=max_d-pt.x+1;
    pb=max_d-elem.rows+pt.x+1;

    cv::copyMakeBorder(elem,elem,pu,pb,pl,pr,cv::BORDER_CONSTANT,cv::Scalar(0));

    pt.x=pt.x+pu;
    pt.y=pt.y+pl;

    pt2.x=pt2.x+pu;
    pt2.y=pt2.y+pl;

    result.pt=cv::Point(round(pt.x),round(pt.y));

    //now pu, pl, pb, and pr are extra lines to add to original map for convolution!!!
    //(different and independent meaning) -> thus they are only an approximation


    pu=pt.x+1;
    pb=elem.rows-pt.x+1;
    pl=pt.y+1;
    pr=elem.cols-pt.y+1;

    result.pl=pl;
    result.pr=pr;
    result.pb=pb;
    result.pu=pu;

    cv::Point2f pt2_f;

    for(int i=0;i<res;i++)
    {
        cv::Mat elem_t;
        cv::Mat r = cv::getRotationMatrix2D(cv::Point2f(pt.y,pt.x), -orient+(360.0/res*i), scale);

        cv::warpAffine(elem, elem_t, r, cv::Size(elem.rows, elem.cols),cv::INTER_LINEAR);

        result.elems.push_back(elem_t);

        cv::Point2f pt2_t=r*cv::Point2f(pt2.y,pt2.x);

        result.pt2.push_back(cv::Point(round(pt2_t.y),round(pt2_t.x)));

        if(i==0)
            pt2_f=cv::Point2f(pt2_t.y,pt2_t.x);
    }

    pt2=pt2_f;

    return result;
}

Elem multiSensor(cv::Mat elem, cv::Point2f pt, double orient ,double scale, int res, cv::Point2f pt2)
{
    Elem result;

    int pl, pr, pu, pb; //add lines for elem to deal with scale and rotation

    pu=elem.rows/2*max(0.0,scale-1);
    pb=elem.rows/2*max(0.0,scale-1);
    pl=elem.cols/2*max(0.0,scale-1);
    pr=elem.cols/2*max(0.0,scale-1);

    cv::copyMakeBorder(elem,elem,pu,pb,pl,pr,cv::BORDER_CONSTANT,cv::Scalar(0));

    pt.x=pt.x+pu;
    pt.y=pt.y+pl;

    pt2.x=pt2.x+pu;
    pt2.y=pt2.y+pl;


    int max_r=max(max(pt.x,elem.rows-pt.x),max(pt2.x,elem.rows-pt2.x)), max_c=max(max(pt.y,elem.cols-pt.y),max(pt2.y,elem.cols-pt2.y));

    int max_d=(int)(sqrt(max_r*max_r+max_c*max_c));

    pl=max_d-pt.y+1;
    pr=max_d-elem.cols+pt.y+1;
    pu=max_d-pt.x+1;
    pb=max_d-elem.rows+pt.x+1;

    cv::copyMakeBorder(elem,elem,pu,pb,pl,pr,cv::BORDER_CONSTANT,cv::Scalar(0));

    pt.x=pt.x+pu;
    pt.y=pt.y+pl;

    pt2.x=pt2.x+pu;
    pt2.y=pt2.y+pl;

    result.pt=cv::Point(round(pt2.x),round(pt2.y));

    //now pu, pl, pb, and pr are extra lines to add to original map for convolution!!!
    //(different and independent meaning) -> thus they are only an approximation


    pu=pt.x+1;
    pb=elem.rows-pt.x+1;
    pl=pt.y+1;
    pr=elem.cols-pt.y+1;

    result.pl=pl;
    result.pr=pr;
    result.pb=pb;
    result.pu=pu;

    cv::Mat r = cv::getRotationMatrix2D(cv::Point2f(pt.y,pt.x), -orient, scale);

    cv::Mat elem_t;

    cv::warpAffine(elem, elem_t, r, cv::Size(elem.rows, elem.cols),cv::INTER_LINEAR);


    for(int i=0;i<res;i++)
    {
        cv::Mat elem_f;
        r = cv::getRotationMatrix2D(cv::Point2f(pt2.y,pt2.x), (360.0/res*i), 1.0);

        cv::warpAffine(elem_t, elem_f, r, cv::Size(elem.rows, elem.cols),cv::INTER_LINEAR);

        result.elems.push_back(elem_f);

        cv::Point2f pt_f=r*cv::Point2f(pt.y,pt.x);

        result.pt2.push_back(cv::Point(round(pt_f.y),round(pt_f.x)));
    }

    return result;
}

vector<cv::Mat> multiErosion(cv::Mat map, Elem robot_or)
{
    vector<cv::Mat> result;
    cv::Mat map_n=map.clone();


    for(unsigned int i=0;i<robot_or.elems.size();i++)
    {
        cv::Mat temp;
        cv::erode( map_n, temp, robot_or.elems[i],cv::Point(robot_or.pt.y,robot_or.pt.x),1, cv::BORDER_CONSTANT,cv::Scalar(0));

        result.push_back(temp);
    }

    return result;
}


vector<cv::Mat> multiDilation(vector<cv::Mat> map_er, Elem robot_or )
{
    vector<cv::Mat> result;

    cv::Point pr;
    pr.y=robot_or.elems[0].cols-1-robot_or.pt.y;
    pr.x=robot_or.elems[0].rows-1-robot_or.pt.x;

    double rob=cv::sum(robot_or.elems[0])[0];

    for(unsigned int i=0;i<robot_or.elems.size();i++)
    {
        cv::Mat temp, elem_t;
        flip(robot_or.elems[i], elem_t, -1);

        if(rob>0)
            cv::dilate(map_er[i], temp, elem_t, cv::Point(pr.y, pr.x) );
        else
            temp=cv::Mat::zeros(map_er[i].rows, map_er[i].cols, CV_8UC1);

        result.push_back(temp);
    }

    return result;
}

void multiMerge(Elem robot_or, Elem& sensor_or, Elem& result, Elem& rev)
{
    result.pb=robot_or.pb;
    result.pu=robot_or.pu;
    result.pl=robot_or.pl;
    result.pr=robot_or.pr;
    result.pt=robot_or.pt;
    result.pt2=robot_or.pt2;
    result.elems.resize(robot_or.elems.size());

    for(unsigned int i=0; i<robot_or.elems.size();i++)
    {
        result.elems[i]=robot_or.elems[i].clone();

        for(int j=0; j<robot_or.elems[i].rows;j++)
        {
            for(int k=0; k<robot_or.elems[i].cols;k++)
            {
                if(robot_or.elems[i].at<uchar>(j,k)!=0)
                {
                    int x=j-robot_or.pt.x+sensor_or.pt.x;
                    int y=k-robot_or.pt.y+sensor_or.pt.y;

                    if(x>=0 && x<sensor_or.elems[i].rows && y>=0 && y<sensor_or.elems[i].cols)
                    {
                        if(sensor_or.elems[i].at<uchar>(x,y)==0)
                        {
                            result.elems[i].at<uchar>(j,k)=0;
                        }
                    }
                    else
                    {
                        result.elems[i].at<uchar>(j,k)=0;
                    }
                }
            }
        }
    }

    rev.pb=robot_or.pb;
    rev.pu=robot_or.pu;
    rev.pl=robot_or.pl;
    rev.pr=robot_or.pr;
    rev.pt=sensor_or.pt;
    rev.pt2=sensor_or.pt2;
    rev.elems.resize(sensor_or.elems.size());

    FindMax<float> maxd_center, maxd_sensor;

    for(unsigned int i=0; i<sensor_or.elems.size();i++)
    {
        rev.elems[i]=sensor_or.elems[i].clone();

        for(int j=0; j<sensor_or.elems[i].rows;j++)
        {
            for(int k=0; k<sensor_or.elems[i].cols;k++)
            {
                if(sensor_or.elems[i].at<uchar>(j,k)!=0)
                {
                    float dist_c=(j-sensor_or.pt.x)*(j-sensor_or.pt.x)+(k-sensor_or.pt.y)*(k-sensor_or.pt.y);
                    maxd_center.iter( sqrt(dist_c) );
                    float dist_s=(j-sensor_or.pt2[i].x)*(j-sensor_or.pt2[i].x)+(k-sensor_or.pt2[i].y)*(k-sensor_or.pt2[i].y);
                    maxd_sensor.iter( sqrt(dist_s) );

                    int x=j-sensor_or.pt.x+robot_or.pt.x;
                    int y=k-sensor_or.pt.y+robot_or.pt.y;

                    if(x>=0 && x<robot_or.elems[i].rows && y>=0 && y<robot_or.elems[i].cols)
                    {
                        if(robot_or.elems[i].at<uchar>(x,y)!=0)
                        {
                            rev.elems[i].at<uchar>(j,k)=0;
                        }
                    }
                }
            }
        }
    }

    rev.pb=(int)(round(maxd_center.getVal())+1);
    rev.pr=(int)(round(maxd_sensor.getVal())+1);

    sensor_or.pb=(int)(round(maxd_center.getVal())+1);
    sensor_or.pr=(int)(round(maxd_sensor.getVal())+1);
}
