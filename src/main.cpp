#include <Eigen/Geometry>
#include "estimationNode.hpp"
#include <string>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gpsimu_odom");
    ros::NodeHandle nh;

    try
    {
        gpsimu_odom::estimationNode gpsimu_odom(nh);
        ros::spin();
    }
    catch(const std::exception &e)
    {
        ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
        return 1;
    }
    return 0;
}