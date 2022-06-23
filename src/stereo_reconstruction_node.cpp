#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <signal.h>
#include <cstdlib>
#include <unistd.h>

#include "stereo_reconstruction.h"
#include "stereo_reconstruction_node.h"


StereoReconstructionNode::StereoReconstructionNode(const std::string node_name, rclcpp::NodeOptions options)
: Node(node_name, options)
{
    std::cerr << "StereoReconstructionNode Starting.." << std::endl;

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("cpp_stereo_reconstruction");
    calibration_file = package_share_directory + "/calibration/calib_params_stereo.xml";
    std::cerr << "calibration_file " << calibration_file << std::endl;

    std::string camera_left_topic = this->declare_parameter<std::string>("subscribers.camera_left", "/camera_left/raw_frame");
    std::string camera_right_topic = this->declare_parameter<std::string>("subscribers.camera_right", "/camera_right/raw_frame");

    std::map<std::string, int> reconstruction_parameters;
    reconstruction_parameters["numDisparities"] = 5*16;
    reconstruction_parameters["blockSize"] = 4*2 + 5;
    reconstruction_parameters["preFilterType"] = 0;
    reconstruction_parameters["preFilterSize"] = 6*2 + 5;
    reconstruction_parameters["preFilterCap"] = 16;
    reconstruction_parameters["textureThreshold"] = 12;
    reconstruction_parameters["uniquenessRatio"] = 12;
    reconstruction_parameters["speckleRange"] = 13;
    reconstruction_parameters["speckleWindowSize"] = 5*2;
    reconstruction_parameters["disp12MaxDiff"] = 3;
    reconstruction_parameters["minDisparity"] = 4;

    stereo_reconstruction = std::make_shared<StereoReconstruction>(calibration_file, reconstruction_parameters);

    pcl_pub = this->create_publisher<PointCloud2>("/pointcloud", 1);

    disparity_pub = this->create_publisher<Image>("/disparity_map", 1);

    right_disparity_pub = this->create_publisher<Image>("/right_disparity_map", 1);

    filter_disparity_pub = this->create_publisher<Image>("/filter_disparity_map", 1);

    frame_left_sub = this->create_subscription<Image>(camera_left_topic, 1, 
        std::bind(&StereoReconstructionNode::callback_frame_left, this, std::placeholders::_1));

    frame_right_sub = this->create_subscription<Image>(camera_right_topic, 1, 
        std::bind(&StereoReconstructionNode::callback_frame_right, this, std::placeholders::_1));

    float hz_pc_computation = 1.0;
    pc_computation_timer = this->create_wall_timer(1000ms / hz_pc_computation, std::bind(&StereoReconstructionNode::generate_pcl_thread, this));
    

}

void StereoReconstructionNode::callback_frame_left(const Image::SharedPtr msg) {
    
    cv_bridge::CvImagePtr cv_ptr;
        
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    current_frame_left = cv_ptr->image;
}

void StereoReconstructionNode::callback_frame_right(const Image::SharedPtr msg) {
    
    cv_bridge::CvImagePtr cv_ptr;
        
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    current_frame_right = cv_ptr->image;
    
}
     

void StereoReconstructionNode::generate_pcl_thread(){

    if(current_frame_left.empty() or current_frame_right.empty())
    {
        std::cerr << "Waiting for both frame acquisition to start..." << std::endl;
        return;
    }

    else{
        cv::Mat img_left = current_frame_left;
        cv::Mat img_right = current_frame_right;
        
        cv::Mat disparity_map = stereo_reconstruction->disparity_from_stereovision(img_left, img_right);

        cv::Mat disparity_map_image;
        double min, max;
        cv::minMaxLoc(disparity_map, &min, &max);
        disparity_map.convertTo(disparity_map_image, CV_8UC1, 255.0 / max);
 
        sensor_msgs::msg::Image::SharedPtr disp_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", disparity_map_image)
                .toImageMsg();

        disp_msg->header.stamp = rclcpp::Clock().now();
        disp_msg->header.frame_id = "pointcloud";
        disparity_pub->publish(*disp_msg.get());


        cv::Mat right_disparity_map = stereo_reconstruction->right_disparity_from_stereovision(img_left, img_right);
        
        cv::Mat right_disparity_map_image;
        cv::minMaxLoc(right_disparity_map, &min, &max);
        right_disparity_map.convertTo(right_disparity_map_image, CV_8UC1, 255.0/max);

        sensor_msgs::msg::Image::SharedPtr right_disp_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", right_disparity_map_image)
                .toImageMsg();

        right_disp_msg->header.stamp = rclcpp::Clock().now();
        right_disp_msg->header.frame_id = "pointcloud";
        right_disparity_pub->publish(*right_disp_msg.get());
        
        // cv::Mat filter_disparity_map = stereo_reconstruction->filter_disparity(img_left, img_right, disparity_map, right_disparity_map);

        std::pair<cv::Mat, cv::Mat> pcl_output = stereo_reconstruction->pcl_from_disparity(disparity_map, img_left, img_right);

        cv::Mat output_points = pcl_output.first;
        cv::Mat output_colors = pcl_output.second;

        PointCloud2 pclmsg;

        pclmsg.header = disp_msg->header;

        pclmsg.width = output_points.cols;
        pclmsg.height = output_points.rows;

        PointField point_field_x = PointField();
        point_field_x.name = 'x';
        point_field_x.count = 1;
        point_field_x.datatype = PointField::FLOAT32;
        point_field_x.offset = 0;
        PointField point_field_y = PointField();
        point_field_y.name = 'y';
        point_field_y.count = 1;
        point_field_y.datatype = PointField::FLOAT32;
        point_field_y.offset = 4;
        PointField point_field_z = PointField();
        point_field_z.name = 'z';
        point_field_z.count = 1;
        point_field_z.datatype = PointField::FLOAT32;
        point_field_z.offset = 8;

        pclmsg.fields = {point_field_x, point_field_y, point_field_z};

        pclmsg.is_bigendian = false;
        pclmsg.is_dense = false;
        pclmsg.point_step = 12;
        pclmsg.row_step = pclmsg.width * pclmsg.point_step;

        std::vector<uchar> array;
        array.assign((uchar*)output_points.datastart, (uchar*)output_points.dataend);
        pclmsg.data = array;
        
        pcl_pub->publish(pclmsg);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();

    rclcpp::NodeOptions node_options;

    node_options.use_intra_process_comms(true);
    auto node = std::make_shared<StereoReconstructionNode>("stereo_reconstruction", node_options);

    executor->add_node(node);
    executor->spin(); 

    rclcpp::shutdown();

    return 0;
}
