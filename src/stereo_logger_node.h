#include <string>
#include <thread>

/* ROS2 Stuff */
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sensor_msgs/msg/image.hpp>
#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>

#ifndef STEREO_LOGGER_NODE_HEADER_FILE
#define STEREO_LOGGER_NODE_HEADER_FILE

using namespace std::chrono_literals;

class StereoLoggerNode : public rclcpp::Node {

    using Image = sensor_msgs::msg::Image;

public:

    StereoLoggerNode(const std::string node_name, rclcpp::NodeOptions options);

private:

    void callback_frame_left(const Image::SharedPtr msg);

    void callback_frame_right(const Image::SharedPtr msg);

    void store_images();

    std::string storage_folder;
    std::string calibration_file;
    int incremental_index;

    cv::Mat current_frame_left, current_frame_right;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr frame_left_sub{nullptr};
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr frame_right_sub{nullptr};

    rclcpp::TimerBase::SharedPtr store_image_timer{nullptr};
};

#endif