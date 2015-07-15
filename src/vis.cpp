#include "vis_transf.hpp"
#include "map_transform/ParametersConfig.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "visibility", ros::init_options::AnonymousName);

    ros::NodeHandle nh("~");

    Vis_transf<map_transform::ParametersConfig> vis(nh);

    ros::Rate loop_rate(10);


    while (ros::ok())
    {
        vis.update();

        ros::spinOnce();

        vis.transf();

        vis.transf_pos();

        vis.show();

        vis.publish();

        loop_rate.sleep();
    }


    return 0;
}
