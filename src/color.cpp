#include "color.hpp"
#include "vector_utils.hpp"

using namespace std;

cv::Mat printPoint(cv::Mat img, cv::Point2i pos, unsigned char* color)
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

cv::Mat color_print(cv::Mat img1, cv::Mat img2, unsigned char* c_b, unsigned char* c_n, unsigned char* c_1, unsigned char* c_2)
{
    cv::Mat result;

    vector<cv::Mat> channels(3);

    channels[0]=(img1&img2/255*c_b[2])+((~img1)&(~img2)/255*c_n[2])+((~img1)&img2/255*c_2[2])+(img1&(~img2)/255*c_1[2]);
    channels[1]=(img1&img2/255*c_b[1])+((~img1)&(~img2)/255*c_n[1])+((~img1)&img2/255*c_2[1])+(img1&(~img2)/255*c_1[1]);
    channels[2]=(img1&img2/255*c_b[0])+((~img1)&(~img2)/255*c_n[0])+((~img1)&img2/255*c_2[0])+(img1&(~img2)/255*c_1[0]);

    cv::merge(channels, result);

    return result;
}

cv::Mat color_print3(cv::Mat img1, cv::Mat img2, cv::Mat img3, unsigned char* c_123, unsigned char* c_12, unsigned char* c_13, unsigned char* c_23, unsigned char* c_1, unsigned char* c_2, unsigned char* c_3, unsigned char* c_0)
{
    cv::Mat result;

    vector<cv::Mat> channels(3);

    channels[0]=(img1&img2&img3/255*c_123[2])+(img1&img2&(~img3)/255*c_12[2])+(img1&(~img2)&img3/255*c_13[2])+((~img1)&img2&img3/255*c_23[2])+(img1&(~img2)&(~img3)/255*c_1[2])+((~img1)&img2&(~img3)/255*c_2[2])+((~img1)&(~img2)&img3/255*c_3[2])+((~img1)&(~img2)&(~img3)/255*c_0[2]);
    channels[1]=(img1&img2&img3/255*c_123[1])+(img1&img2&(~img3)/255*c_12[1])+(img1&(~img2)&img3/255*c_13[1])+((~img1)&img2&img3/255*c_23[1])+(img1&(~img2)&(~img3)/255*c_1[1])+((~img1)&img2&(~img3)/255*c_2[1])+((~img1)&(~img2)&img3/255*c_3[1])+((~img1)&(~img2)&(~img3)/255*c_0[1]);
    channels[2]=(img1&img2&img3/255*c_123[0])+(img1&img2&(~img3)/255*c_12[0])+(img1&(~img2)&img3/255*c_13[0])+((~img1)&img2&img3/255*c_23[0])+(img1&(~img2)&(~img3)/255*c_1[0])+((~img1)&img2&(~img3)/255*c_2[0])+((~img1)&(~img2)&img3/255*c_3[0])+((~img1)&(~img2)&(~img3)/255*c_0[0]);

    cv::merge(channels, result);

    return result;
}
