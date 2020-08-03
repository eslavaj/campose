#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/core/cuda.hpp>


struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    cv::cuda::GpuMat gpu_cameraImg; // contains camera image on gpu
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::cuda::GpuMat gpu_keypoints; // GPU 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    cv::cuda::GpuMat gpu_descriptors; // gpu keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
    cv::cuda::GpuMat gpu_kptMatches; // gpu keypoint matches between previous and current frame
};


#endif /* dataStructures_h */
