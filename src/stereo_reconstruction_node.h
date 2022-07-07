#include <string>
#include <thread>

/* ROS2 Stuff */
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>
#include "stereo_reconstruction.h"

#ifndef STEREO_RECONSTRUCTION_NODE_HEADER_FILE
#define STEREO_RECONSTRUCTION_NODE_HEADER_FILE

using namespace std::chrono_literals;

class StereoReconstructionNode : public rclcpp::Node {

    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using PointField = sensor_msgs::msg::PointField;
    using Image = sensor_msgs::msg::Image;

public:

    StereoReconstructionNode(const std::string node_name, rclcpp::NodeOptions options);

private:

    void callback_frame_left(const Image::SharedPtr msg);

    void callback_frame_right(const Image::SharedPtr msg);

    void generate_pcl_thread();

    void generate_pcl_thread_test_to_calibrate();

    std::shared_ptr<StereoReconstruction> stereo_reconstruction;

    std::string package_share_directory;
    std::string calibration_file;

    cv::Mat current_frame_left, current_frame_right;

    rclcpp::Publisher<PointCloud2>::SharedPtr pcl_pub{nullptr};
    rclcpp::Publisher<Image>::SharedPtr disparity_pub{nullptr};
    rclcpp::Publisher<Image>::SharedPtr right_disparity_pub{nullptr};
    rclcpp::Publisher<Image>::SharedPtr filter_disparity_pub{nullptr};
    

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr frame_left_sub{nullptr};
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr frame_right_sub{nullptr};

    rclcpp::TimerBase::SharedPtr pc_computation_timer{nullptr};
};

#endif