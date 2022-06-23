#include <iostream>
#include "stereo_reconstruction.h"

using namespace cv;

StereoReconstruction::StereoReconstruction(std::string file_storage, std::map<std::string, int> reconstruction_parameters)
{
    FileStorage fs(file_storage, FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "Failed to open " << file_storage << std::endl;
        return;
    }

    fs["stereoMapL_x"] >> stereoMapL_x;
    fs["stereoMapL_y"] >> stereoMapL_y;
    fs["stereoMapR_x"] >> stereoMapR_x;
    fs["stereoMapR_y"] >> stereoMapR_y;

    fs["q"] >> Q;

    stereo = StereoBM::create();
    
    stereo->setNumDisparities(reconstruction_parameters["numDisparities"]);
    stereo->setBlockSize(reconstruction_parameters["blockSize"]);
    stereo->setPreFilterType(reconstruction_parameters["preFilterType"]);
    stereo->setPreFilterSize(reconstruction_parameters["preFilterSize"]);
    stereo->setPreFilterCap(reconstruction_parameters["preFilterCap"]);
    stereo->setTextureThreshold(reconstruction_parameters["textureThreshold"]);
    stereo->setUniquenessRatio(reconstruction_parameters["uniquenessRatio"]);
    stereo->setSpeckleRange(reconstruction_parameters["speckleRange"]);
    stereo->setSpeckleWindowSize(reconstruction_parameters["speckleWindowSize"]);
    stereo->setDisp12MaxDiff(reconstruction_parameters["disp12MaxDiff"]);
    stereo->setMinDisparity(reconstruction_parameters["minDisparity"]);

    minDisparity = reconstruction_parameters["minDisparity"];
    numDisparities = reconstruction_parameters["numDisparities"];

    right_matcher = createRightMatcher(stereo);

}

Mat StereoReconstruction::disparity_from_stereovision(Mat img_left, Mat img_right){  

    Mat img_left_gray, img_right_gray;        
    
    cvtColor(img_left, img_left_gray, cv::COLOR_RGB2GRAY);
    cvtColor(img_right, img_right_gray, cv::COLOR_RGB2GRAY);

    Mat img_left_nice, img_right_nice; 

    remap(img_left_gray, img_left_nice, stereoMapL_x, stereoMapL_y, INTER_LANCZOS4, BORDER_CONSTANT,  Scalar(0, 0, 0) );

    remap(img_right_gray, img_right_nice, stereoMapR_x, stereoMapR_y, INTER_LANCZOS4, BORDER_CONSTANT,  Scalar(0, 0, 0) );

    Mat disparity_map;
    stereo->compute(img_left_nice, img_right_nice, disparity_map);

    return disparity_map;
}

Mat StereoReconstruction::right_disparity_from_stereovision(Mat img_left, Mat img_right){  

    Mat img_left_gray, img_right_gray;        
    
    cvtColor(img_left, img_left_gray, cv::COLOR_RGB2GRAY);
    cvtColor(img_right, img_right_gray, cv::COLOR_RGB2GRAY);

    Mat img_left_nice, img_right_nice; 

    remap(img_left_gray, img_left_nice, stereoMapL_x, stereoMapL_y, INTER_LANCZOS4, BORDER_CONSTANT,  Scalar(0, 0, 0) );

    remap(img_right_gray, img_right_nice, stereoMapR_x, stereoMapR_y, INTER_LANCZOS4, BORDER_CONSTANT,  Scalar(0, 0, 0) );

    Mat disparity_map;
    right_matcher->compute(img_right_nice, img_left_nice, disparity_map);

    disparity_map = -1 * disparity_map;

    return disparity_map;
}

std::pair<cv::Mat, cv::Mat> StereoReconstruction::pcl_from_disparity(cv::Mat disparity_map, cv::Mat img_left, cv::Mat img_right){

    disparity_map.convertTo(disparity_map, CV_32FC1); 
    disparity_map = disparity_map / 16.0;

    cv::Mat points_3D(disparity_map.size(),CV_32FC1);
    reprojectImageTo3D(disparity_map, points_3D, Q, false, CV_32FC1);

    cv::Mat colors;
    cv::cvtColor(img_right, colors, cv::COLOR_BGR2RGB);

    double min, max;
    cv::minMaxLoc(disparity_map, &min, &max);

    cv::Mat mask = disparity_map > min;

    cv::Mat output_points;
    cv::Mat output_colors;

    points_3D.copyTo(output_points, mask);
    colors.copyTo(output_colors, mask);

    return std::make_pair(output_points, output_colors);
}


cv::Mat StereoReconstruction::filter_disparity(cv::Mat left_image, cv::Mat right_image, cv::Mat left_disp, cv::Mat right_disp){


   	Mat filtered_disp, conf_map;
	
    /* Filter
        * MD calculated by the respective match instances, just as the 
        * left image is passed to the filter. 
        * Note that we are using the original image to guide the filtering 
        * process.
    */

    wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);

    double lambda, sigma; //Post-filter parameters
    lambda = 5000.0;
    sigma = 1.0;
    wls_filter->setLambda(lambda);
    wls_filter->setSigmaColor(sigma);

    wls_filter->filter(left_disp, left_image, filtered_disp, right_disp, cv::Rect(), right_image);
    
	// conf_map = Mat(left_disp.rows, left_disp.cols, CV_8U);
	// conf_map = Scalar(255);
    // conf_map = wls_filter->getConfidenceMap();
    
    return filtered_disp;
    
}
