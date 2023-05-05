#include <ros/ros.h>
#include "bilateral_filter/bilateral_filter.hpp"

int main(int argc, char **argv)
{
    // node
    ros::init(argc, argv, "bilateral_filter_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // code
    bilateral_filter::BilateralFilterRos node(nh, nh_private);
    node.setupROSparams();
    node.setupSubAndPub();

    // spiner
    ros::spin();

    // return
    return 0;
}