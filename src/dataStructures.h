#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/core/cuda.hpp>


struct DataFrame { // represents the available sensor information at the same time instance
    

	DataFrame()
	{
		cv::Mat projection1(3, 4, CV_64F, 0.);    // the 3x4 projection matrix
		cv::Mat diag(cv::Mat::eye(3, 3, CV_64F));
		diag.copyTo(projection1(cv::Rect(0, 0, 3, 3)));
		projectionMatrix = projection1.clone();
	};

    cv::Mat cameraImg; // camera image
    cv::cuda::GpuMat gpu_cameraImg; // contains camera image on gpu
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    std::vector<cv::Point2f> refinedPointsCurr; //refined points of this frame after refining according matching
    std::vector<cv::Point2f> refinedPointsPrev; //refined points of the previous frame corresponding to refined points of this frame
    std::vector<cv::Vec2d> undistortedPointsCurr; //Undistorted points on current frame
    std::vector<cv::Vec2d> undistortedPointsPrev; //Undistorted points on previous frame
    std::vector<cv::Vec3d> points3D; //3D points obtained after triangulation using preious and current frame

    cv::cuda::GpuMat gpu_keypoints; // GPU 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    cv::cuda::GpuMat gpu_descriptors; // gpu keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
    cv::cuda::GpuMat gpu_kptMatches; // gpu keypoint matches between previous and current frame

    cv::Mat essentialMatrix; //Rotation matrix relative to the previous frame
    cv::Mat rotationMatrix; //Rotation matrix relative to the previous frame
    cv::Mat translationVector; //Translation vector relative to the previous frame
    cv::Mat projectionMatrix; //Projection matrix that includes rotation and translation information

};


#endif /* dataStructures_h */
