#pragma once
#define PCL_NO_PRECOMPILE // must define PCL_NO_PRECOMPILE before including any PCL templates when using custom point type

#include <ros/ros.h>
#include "point_type/HesaiPointT.hpp"
#include "point_type/GazeboPointT.hpp"
#include <pcl_conversions/pcl_conversions.h>

namespace bilateral_filter
{
    class BilateralFilterRos
    {
    public:
        BilateralFilterRos(ros::NodeHandle nh, ros::NodeHandle nh_private);
        ~BilateralFilterRos();

        void setupROSparams();
        void setupSubAndPub();

    private:
        template <typename T> void readParamsWithPrompt_private(std::string name, T &parameter, std::string text_prompt);
        template <typename T> void readParamsWithPrompt(std::string name, T &parameter, std::string text_prompt);
        void callback(const sensor_msgs::PointCloud2::ConstPtr &msg_pointcloud);
        void readData(const sensor_msgs::PointCloud2::ConstPtr &msg_pointcloud);
        void runAlgorithm();
        void publish();

        // node handle
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        // subcriber and publisher
        ros::Subscriber sub_input_;
        ros::Publisher pub_output_;

        // parameters
        std::string topic_input_; // subscriber and publisher
        std::string topic_output_;
        double bilateral_width_; // bilateral filtering
        double bilateral_sigma_d_;
        double bilateral_sigma_i_;

        
        // data
        std_msgs::Header msg_input_header_;
        pcl::PointCloud<HesaiPointT> pointcloud_input_;
        pcl::PointCloud<HesaiPointT> pointcloud_output_;
    };
}