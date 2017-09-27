#include "color.hpp"
#include "vector_utils.hpp"

using namespace std;

cv::Mat printPoint(cv::Mat img, cv::Point pos, unsigned char* color)
{
    cv::Mat ret;

    if(img.type()==CV_8UC1)
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

        cv::merge(channels, ret);
    }
    else if(img.type()==CV_8UC3)
    {
        ret=img.clone();
        for(int k=0;k<3;k++)
        {
            for(int i=(pos.x-1);i<=(pos.x+1);i++)
            {
                for(int j=(pos.y-1);j<=(pos.y+1);j++)
                {
                    ret.at<cv::Vec3b>(boundPos(i,img.rows),boundPos(j,img.cols))[k]=color[2-k];
                }
            }
        }
    }
    else
    {
        ret=img.clone();
    }

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

cv::Mat color_print_expansion(cv::Mat img0, cv::Mat img1, cv::Mat img3, cv::Mat img2, cv::Point target, unsigned char* c_b, unsigned char* c_w, unsigned char* c_n, unsigned char* c_v, unsigned char* c_o, unsigned char* c_c, unsigned char* c_cf, unsigned char* c_g, unsigned char* c_p, unsigned char* c_t)
{
    cv::Mat result;

    vector<cv::Mat> channels(3);
    channels[0]=cv::Mat::zeros(img1.rows, img1.cols, CV_8UC1);
    channels[1]=cv::Mat::zeros(img1.rows, img1.cols, CV_8UC1);
    channels[2]=cv::Mat::zeros(img1.rows, img1.cols, CV_8UC1);

    cv::Point perc(-1,-1);

    for(unsigned int r=0; r<img1.rows; r++){
        for(unsigned int c=0; c<img1.cols; c++){
            if(img0.at<uchar>(r,c)!=0){
                if(img1.at<uchar>(r,c)==0){
                    if(img3.at<uchar>(r,c)==0){
                        channels[0].at<uchar>(r,c)=c_n[2];
                        channels[1].at<uchar>(r,c)=c_n[1];
                        channels[2].at<uchar>(r,c)=c_n[0];
                    }
                    else{
                        channels[0].at<uchar>(r,c)=c_v[2];
                        channels[1].at<uchar>(r,c)=c_v[1];
                        channels[2].at<uchar>(r,c)=c_v[0];
                    }
                }
                else{
                    if(img2.at<uchar>(r,c)==0){
                        channels[0].at<uchar>(r,c)=c_w[2];
                        channels[1].at<uchar>(r,c)=c_w[1];
                        channels[2].at<uchar>(r,c)=c_w[0];
                    }
                    else if(img2.at<uchar>(r,c)==OPEN_COLOR){
                        channels[0].at<uchar>(r,c)=c_o[2];
                        channels[1].at<uchar>(r,c)=c_o[1];
                        channels[2].at<uchar>(r,c)=c_o[0];
                    }
                    else if(img2.at<uchar>(r,c)==CLOSED_COLOR){
                        channels[0].at<uchar>(r,c)=c_c[2];
                        channels[1].at<uchar>(r,c)=c_c[1];
                        channels[2].at<uchar>(r,c)=c_c[0];
                    }
                    else if(img2.at<uchar>(r,c)==CLOSED_FEAS_COLOR){
                        channels[0].at<uchar>(r,c)=c_cf[2];
                        channels[1].at<uchar>(r,c)=c_cf[1];
                        channels[2].at<uchar>(r,c)=c_cf[0];
                    }
                    else if(img2.at<uchar>(r,c)==GOALS_COLOR){
                        channels[0].at<uchar>(r,c)=c_g[2];
                        channels[1].at<uchar>(r,c)=c_g[1];
                        channels[2].at<uchar>(r,c)=c_g[0];
                    }
                    else if(img2.at<uchar>(r,c)==PERC_COLOR){
                        perc.x=r;
                        perc.y=c;
                    }
                }
            }
            else{
                channels[0].at<uchar>(r,c)=c_b[2];
                channels[1].at<uchar>(r,c)=c_b[1];
                channels[2].at<uchar>(r,c)=c_b[0];
            }
        }
    }

    cv::merge(channels, result);

    result=printPoint(result,target,c_t);
    if(perc.x>=0 && perc.y>=0)
        result=printPoint(result,perc,c_p);

    return result;
}
