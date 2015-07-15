#include "morph.hpp"

#include <opencv2/imgproc/imgproc.hpp>

using namespace std;


Elem multiElem(cv::Mat elem, cv::Point2f pt, double orient ,double scale, int res)
{
    Elem result;

    int pl, pr, pu, pb;

    pu=elem.rows/2*max(0.0,scale-1);
    pb=elem.rows/2*max(0.0,scale-1);
    pl=elem.cols/2*max(0.0,scale-1);
    pr=elem.cols/2*max(0.0,scale-1);

    cv::copyMakeBorder(elem,elem,pu,pb,pl,pr,cv::BORDER_CONSTANT,cv::Scalar(0));

    pt.x=pt.x+pl;
    pt.y=pt.y+pu;


    int max_r=max(pt.x,elem.cols-pt.x), max_c=max(pt.y,elem.rows-pt.y);

    int max_d=(int)(sqrt(max_r*max_r+max_c*max_c));

    pl=max_d-pt.x+1;
    pr=max_d-elem.cols+pt.x+1;
    pu=max_d-pt.y+1;
    pb=max_d-elem.rows+pt.y+1;

    cv::copyMakeBorder(elem,elem,pu,pb,pl,pr,cv::BORDER_CONSTANT,cv::Scalar(0));

    pt.x=pt.x+pl;
    pt.y=pt.y+pu;


    result.pt=cv::Point(pt.x,pt.y);



    pu=pt.y+1;
    pb=elem.rows-pt.y+1;
    pl=pt.x+1;
    pr=elem.cols-pt.x+1;

    result.pl=pl;
    result.pr=pr;
    result.pb=pb;
    result.pu=pu;


    for(int i=0;i<res;i++)
    {
        cv::Mat elem_t;
        cv::Mat r = cv::getRotationMatrix2D(pt, -orient+(360.0/res*i), scale);



        cv::warpAffine(elem, elem_t, r, cv::Size(elem.rows, elem.cols),cv::INTER_LINEAR);

        result.elems.push_back(elem_t);
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
        cv::erode( map_n, temp, robot_or.elems[i], robot_or.pt,1, cv::BORDER_CONSTANT,cv::Scalar(0));

        result.push_back(temp);
    }

    return result;
}


vector<cv::Mat> multiDilation(vector<cv::Mat> map_er, Elem robot_or )
{


    vector<cv::Mat> result;

    cv::Point pr;
    pr.x=robot_or.elems[0].cols-1-robot_or.pt.x;
    pr.y=robot_or.elems[0].rows-1-robot_or.pt.y;

    for(unsigned int i=0;i<robot_or.elems.size();i++)
    {
        cv::Mat temp, elem_t;
        flip(robot_or.elems[i], elem_t, -1);


        cv::dilate(map_er[i], temp, elem_t, pr );

        result.push_back(temp);


    }

    return result;

}
