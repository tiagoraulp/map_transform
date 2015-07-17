#include "visNC_transf.hpp"
#include "clustering.hpp"

#include <opencv2/highgui/highgui.hpp>

using namespace std;

static string filename;

int main(int argc, char **argv)
{

    std::vector<cv::Point3i> x; x.push_back(cv::Point3i(1,1,0));x.push_back(cv::Point3i(1,2,0));
    x.push_back(cv::Point3i(1,3,1));x.push_back(cv::Point3i(1,4,2));x.push_back(cv::Point3i(1,5,3));
    x.push_back(cv::Point3i(1,6,2));x.push_back(cv::Point3i(1,7,1));x.push_back(cv::Point3i(1,9,0));
    x.push_back(cv::Point3i(2,7,5));x.push_back(cv::Point3i(1,1,1));x.push_back(cv::Point3i(1,8,6));
    x.push_back(cv::Point3i(-1,6,2));x.push_back(cv::Point3i(0,0,0));x.push_back(cv::Point3i(0,7,1));
    x.push_back(cv::Point3i(1,1,9));

    std::vector<cv::Point3i> y=cluster_points(x,x.begin()+8,12);

    for(int j=0; j<y.size();j++)
    {
        std::cout<<y[j].x<<" "<<y[j].y<<" "<<y[j].z<<std::endl;
    }

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

    VisNC_transf visNC(nh,robot);

    ros::Rate loop_rate(10);


    while (ros::ok())
    {

        ros::spinOnce();

        visNC.run();

        loop_rate.sleep();
    }


    return 0;
}
