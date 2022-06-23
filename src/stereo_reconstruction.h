#include <string>
#include <map>

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp" 

#ifndef STEREO_RECONSTRUCTION_HEADER_FILE
#define STEREO_RECONSTRUCTION_HEADER_FILE

using namespace cv;
using namespace cv::ximgproc;

class StereoReconstruction {

public:

    StereoReconstruction(std::string file_storage, std::map<std::string, int> reconstruction_parameters);

    cv::Mat disparity_from_stereovision(Mat img_left, Mat img_right);

    cv::Mat right_disparity_from_stereovision(Mat img_left, Mat img_right);

    cv::Mat filter_disparity(cv::Mat left_image, cv::Mat right_image, cv::Mat left_disp, cv::Mat right_disp);

    std::pair<cv::Mat, cv::Mat> pcl_from_disparity(cv::Mat disparity_map, cv::Mat img_left, cv::Mat img_right);

private:

    cv::Mat stereoMapL_x;
    cv::Mat stereoMapL_y;
    cv::Mat stereoMapR_x;
    cv::Mat stereoMapR_y;
    
    cv::Mat Q;

    Ptr<StereoBM> stereo;
	Ptr<DisparityWLSFilter> wls_filter;
    Ptr<StereoMatcher> right_matcher;

    float minDisparity, numDisparities;

};

#endif