#include "visNC_transf.hpp"
#include "map_transform/ParametersncConfig.h"

#include <opencv2/highgui/highgui.hpp>

using namespace std;

static string filename;

int main(int argc, char **argv)
{
    //        vector<int> compression_params;
    //        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    //        compression_params.push_back(9);
    //        try {
    //            cv::imwrite("/home/tiago/map_debug.png", map_debug, compression_params);
    //        }
    //        catch (exception& e)
    //        {
    //          cout  <<"Exception: "<< e.what() << '\n';
    //        }


    ros::init(argc, argv, "visNC");

    if(argc==2)
    {
        filename=string(argv[1]);
    }
    else
        return -1;


    cv::Mat robot=cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);


    if(robot.empty())
        return -2;


    for(int i=0;i<robot.rows;i++)
    {
        for(int j=0;j<robot.cols;j++)
        {
            if (robot.at<uchar>(i,j)>128)
                robot.at<uchar>(i,j)=255;
            else
                robot.at<uchar>(i,j)=0;
        }
    }

    ros::NodeHandle nh("~");

    VisNC_transf<map_transform::ParametersncConfig> visNC(nh,robot);


    ros::Rate loop_rate(10);


    while (ros::ok())
    {
        visNC.update();

        ros::spinOnce();

        visNC.transf();

        visNC.transf_pos();

        visNC.show();

        visNC.publish();

        loop_rate.sleep();
    }


    return 0;
}
