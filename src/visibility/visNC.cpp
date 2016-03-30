#include "visNC_transf.hpp"
#include <opencv2/highgui/highgui.hpp>

using namespace std;

static string filename_r, filename_s;

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

    if(argc==3)
    {
        filename_r=string(argv[1]);
        filename_s=string(argv[2]);
    }
    else
        return -1;


    cv::Mat robot=cv::imread(filename_r, CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat sensor=cv::imread(filename_s, CV_LOAD_IMAGE_GRAYSCALE);


    if(robot.empty() || sensor.empty())
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

    for(int i=0;i<sensor.rows;i++)
    {
        for(int j=0;j<sensor.cols;j++)
        {
            if (sensor.at<uchar>(i,j)>128)
                sensor.at<uchar>(i,j)=255;
            else
                sensor.at<uchar>(i,j)=0;
        }
    }

    ros::NodeHandle nh("~");

    VisNC_transf visNC(nh,robot, sensor);

    ros::Rate loop_rate(10);


    while (ros::ok())
    {

        ros::spinOnce();

        visNC.run(true);

        //visNC.run(false);

        loop_rate.sleep();
    }


    return 0;
}
