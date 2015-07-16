#include "visC_transf.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "visibility", ros::init_options::AnonymousName);

    ros::NodeHandle nh("~");

    VisC_transf vis(nh);

    ros::Rate loop_rate(10);


    while (ros::ok())
    {
        ros::spinOnce();

        vis.run();

        loop_rate.sleep();
    }


    return 0;
}
