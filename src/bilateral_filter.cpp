#include "bilateral_filter/bilateral_filter.hpp"

using namespace bilateral_filter;

BilateralFilterRos::BilateralFilterRos(ros::NodeHandle nh, ros::NodeHandle nh_private) : nh_(nh), nh_private_(nh_private){};
BilateralFilterRos::~BilateralFilterRos() {}

template <typename T>
void BilateralFilterRos::readParamsWithPrompt_private(std::string name, T &parameter, std::string text_prompt)
{
    if (nh_private_.getParam(name, parameter))
    {
        ROS_INFO_STREAM(text_prompt << parameter);
    }
    else
    {
        ROS_ERROR_STREAM("Could not get " << name << "!");
    }
}

template <typename T>
void BilateralFilterRos::readParamsWithPrompt(std::string name, T &parameter, std::string text_prompt)
{
    if (nh_.getParamCached(name, parameter))
    {
        ROS_INFO_STREAM(text_prompt);
    }
    else
    {
        ROS_ERROR_STREAM("Could not get " << name << "!");
    }
}
    
void BilateralFilterRos::setupROSparams()
{
    std::cout << std::endl;
    ROS_INFO_STREAM("[Loading Parameters]");
    
    // subscriber and publisher
    readParamsWithPrompt_private("topic_input", topic_input_, "Input topic: ");
    readParamsWithPrompt_private("topic_output", topic_output_, "Output topic: ");

    // bilateral filtering
    readParamsWithPrompt_private("bilateral_width", bilateral_width_, "Bilateral width: ");
    readParamsWithPrompt_private("bilateral_sigma_d", bilateral_sigma_d_, "Bilateral sigma d: ");
    readParamsWithPrompt_private("bilateral_sigma_i", bilateral_sigma_i_, "Bilateral sigma i: ");
};

void BilateralFilterRos::setupSubAndPub()
{
    std::cout << std::endl;
    ROS_INFO_STREAM("[Subscriber and Publisher]");

    // subscriber
    sub_input_ = nh_.subscribe(topic_input_, 1, &BilateralFilterRos::BilateralFilterRos::callback, this);
    ROS_INFO_STREAM("Subscribing to: " << sub_input_.getTopic());
    
    // publisher
    pub_output_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_output_, 0, true);
    ROS_INFO_STREAM("Publishing to: " << pub_output_.getTopic());
}

void BilateralFilterRos::callback(const sensor_msgs::PointCloud2::ConstPtr &msg_input_constptr)
{
    // read data
    readData(msg_input_constptr);
    
    // algorithm
    runAlgorithm();

    // publish
    publish();
};

void BilateralFilterRos::readData(const sensor_msgs::PointCloud2::ConstPtr &msg_input_constptr)
{
    // topic
    msg_input_header_ = msg_input_constptr->header; // header
    pcl::fromROSMsg(*msg_input_constptr, pointcloud_input_); // data
}

void BilateralFilterRos::runAlgorithm()
{
    // resize output
    pointcloud_output_.width = pointcloud_input_.width;
    pointcloud_output_.height = pointcloud_input_.height;
    pointcloud_output_.resize(pointcloud_input_.width * pointcloud_input_.height);

    // obtain range image
    std::vector<std::vector<float>> range_image;
    BilateralFilterRos::obtainRangeImage(pointcloud_input_, range_image);

    // perform filtering
    for (int i = bilateral_width_; i < pointcloud_input_.width-bilateral_width_; i++){
        for (int j = 0; j < pointcloud_input_.height; j++){

            // nan check
            bool contain_nan = false;
            for (int k = -bilateral_width_; k<bilateral_width_; k++) {if (isnan(range_image.at(i+k).at(j))) {contain_nan = true;};};
            if (contain_nan) {continue;};

            // filtered value
            float num = 0.f; // numerator
            float den = 0.f; // denominator
            for (int k = -bilateral_width_; k<bilateral_width_; k++){
                if (isnan(range_image.at(i+k).at(j))) {continue;};

                float diff_d = tanf(abs(k)/180.f*M_PI)*range_image.at(i).at(j);
                float diff_i = range_image.at(i).at(j) - range_image.at(i+k).at(j);
                float gauss_d = expf(-1.f/2.f*powf(diff_d/bilateral_sigma_d_, 2.f));
                float gauss_i = expf(-1.f/2.f*powf(diff_i/bilateral_sigma_i_, 2.f));

                num += gauss_d * gauss_i * range_image.at(i+k).at(j);
                den += gauss_d * gauss_i;
            };
            if (num == 0) {continue;};
            float new_range = num/den;

            // convert range to xyz
            pointcloud_output_.at(i, j).x = new_range / range_image.at(i).at(j) * pointcloud_input_.at(i, j).x; 
            pointcloud_output_.at(i, j).y = new_range / range_image.at(i).at(j) * pointcloud_input_.at(i, j).y; 
            pointcloud_output_.at(i, j).z = new_range / range_image.at(i).at(j) * pointcloud_input_.at(i, j).z; 
            pointcloud_output_.at(i, j).intensity = pointcloud_input_.at(i, j).intensity; 
            pointcloud_output_.at(i, j).timestamp = pointcloud_input_.at(i, j).timestamp; 
            pointcloud_output_.at(i, j).ring = pointcloud_input_.at(i, j).ring;
        }
    }    
}

void BilateralFilterRos::obtainRangeImage(const pcl::PointCloud<HesaiPointT> &cloud_organized, std::vector<std::vector<float>> &range_image)
{    
    // resize
    int ROW = cloud_organized.width;
    int COL = cloud_organized.height;
    range_image.resize(ROW, std::vector<float>(COL));

    // assign
    for (int i = 0; i < cloud_organized.width; i++){
        for (int j = 0; j < cloud_organized.height; j++){
            range_image.at(i).at(j) = cloud_organized.at(i, j).getVector3fMap().norm();
        }
    }    
}

void BilateralFilterRos::publish()
{   
    // publish
    sensor_msgs::PointCloud2 msg_output_;
    pcl::toROSMsg(pointcloud_output_, msg_output_);
    msg_output_.header = msg_input_header_;
    pub_output_.publish(msg_output_);
}