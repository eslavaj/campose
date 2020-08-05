/*
 * Triangulate.cpp
 *
 *  Created on: Aug 3, 2020
 *      Author: jeslava
 */

#include "CameraPoseEstimator.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/viz.hpp>

//#include "calibration_matrix.hpp"
#include "triangulate.h"

void CameraPoseEstimator::calcCameraPose()
{
	if(m_dataFrameBuffer.size()>1)
	{
		cv::Mat inliers;
		(m_dataFrameBuffer.end() - 1)->essentialMatrix =
				cv::findEssentialMat(
				(m_dataFrameBuffer.end() - 1)->refinedPointsPrev, (m_dataFrameBuffer.end() - 1)->refinedPointsCurr,
				m_A_calib,	          // intrinsic parameters
				cv::RANSAC, 0.999, 1.0, // RANSAC method
				inliers);             // extracted inliers

		int numberOfPts(cv::sum(inliers)[0]);
		std::cout << "#5 : Number of inliers according to Essential matrix: " << numberOfPts << std::endl;
		//std::cout << "Essential matrix: " << (m_dataFrameBuffer.end() - 1)->essentialMatrix << std::endl;

		// recover relative camera pose from essential matrix
		//cv::Mat rotation, translation;
		cv::recoverPose((m_dataFrameBuffer.end() - 1)->essentialMatrix,   // the essential matrix
				(m_dataFrameBuffer.end() - 1)->refinedPointsPrev, (m_dataFrameBuffer.end() - 1)->refinedPointsCurr,        // the matched keypoints
				m_A_calib,            // matrix of intrinsics
				(m_dataFrameBuffer.end() - 1)->rotationMatrix, (m_dataFrameBuffer.end() - 1)->translationVector,   // estimated motion
				//cv::noArray());                // inliers matches
				inliers);

		// compose projection matrix from R,T
		cv::Mat projectionM(3, 4, CV_64F);        // the 3x4 projection matrix
		(m_dataFrameBuffer.end() - 1)->rotationMatrix.copyTo(projectionM(cv::Rect(0, 0, 3, 3)));
		(m_dataFrameBuffer.end() - 1)->translationVector.copyTo(projectionM.colRange(3, 4));

		projectionM.copyTo((m_dataFrameBuffer.end() - 1)->projectionMatrix);


		numberOfPts = cv::sum(inliers)[0];
		std::cout << "#6 : Number of inliers after recovering pose: " << numberOfPts << std::endl;

		/*Check if camera has moved enough, if not then discard this frame*/
		if(numberOfPts<8)
		{
			m_dataFrameBuffer.pop_back();
			return;
		}

		// to contain the inliers
		std::vector<cv::Vec2d> inlierPts1;
		std::vector<cv::Vec2d> inlierPts2;

		// create inliers input point vector for triangulation
		for (int i = 0; i < inliers.rows; i++) {
			if (inliers.at<uchar>(i)) {
				inlierPts1.push_back(cv::Vec2d((m_dataFrameBuffer.end() - 1)->refinedPointsPrev[i].x, (m_dataFrameBuffer.end() - 1)->refinedPointsPrev[i].y));
				inlierPts2.push_back(cv::Vec2d((m_dataFrameBuffer.end() - 1)->refinedPointsCurr[i].x, (m_dataFrameBuffer.end() - 1)->refinedPointsCurr[i].y));
			}
		}

		// undistort and normalize the image points
		std::vector<cv::Vec2d> points1u;
		cv::undistortPoints(inlierPts1, points1u, m_A_calib, m_D_calib);
		std::vector<cv::Vec2d> points2u;
		cv::undistortPoints(inlierPts2, points2u, m_A_calib, m_D_calib);

		/*If the previous frame does not have projection matrix then assume a generic matrix*/
		/*
		if((m_dataFrameBuffer.end() - 2)->projectionMatrix.cols == 0)
		{
			cv::Mat projection_tmp(3, 4, CV_64F, 0.);    // the 3x4 projection matrix
			cv::Mat diag(cv::Mat::eye(3, 3, CV_64F));
			diag.copyTo(projection_tmp(cv::Rect(0, 0, 3, 3)));
			projection_tmp.copyTo((m_dataFrameBuffer.end() - 2)->projectionMatrix);
		}*/


		// triangulation
		std::vector<cv::Vec3d> points3D;
		triangulate((m_dataFrameBuffer.end() - 2)->projectionMatrix, (m_dataFrameBuffer.end() - 1)->projectionMatrix, points1u, points2u, points3D);

		(m_dataFrameBuffer.end() - 1)->undistortedPointsCurr = points2u;
		(m_dataFrameBuffer.end() - 1)->undistortedPointsPrev = points1u;
		(m_dataFrameBuffer.end() - 1)->points3D = points3D;

	}
}



void CameraPoseEstimator::visualize()
{
	if(m_dataFrameBuffer.size()>1)
	{
		// -------------------
		// Create a viz window
		//cv::viz::Viz3d visualizer("Viz window");
		m_visualizer.setBackgroundColor(cv::viz::Color::white());

		/// Construct the scene
		// Create one virtual camera
		cv::viz::WCameraPosition cam1(m_A_calib33f,  // matrix of intrinsics
				(m_dataFrameBuffer.end() - 2)->cameraImg,                             // image displayed on the plane
				1.0,                                // scale factor
				cv::viz::Color::black());

		// Create a second virtual camera
		cv::viz::WCameraPosition cam2(m_A_calib33f,  // matrix of intrinsics
				(m_dataFrameBuffer.end() - 1)->cameraImg,                             // image displayed on the plane
				1.0,                                // scale factor
				cv::viz::Color::black());

		// choose one point for visualization
		cv::Vec3d testPoint = triangulate((m_dataFrameBuffer.end() - 2)->projectionMatrix, (m_dataFrameBuffer.end() - 1)->projectionMatrix,
				(m_dataFrameBuffer.end() - 1)->undistortedPointsPrev[UNDISTORTED_POINT_NBR],
				(m_dataFrameBuffer.end() - 1)->undistortedPointsCurr[UNDISTORTED_POINT_NBR]);

		cv::viz::WSphere point3D(testPoint, 0.05, 10, cv::viz::Color::red());

		// its associated line of projection
		double lenght(10.);
		cv::viz::WLine line1(cv::Point3d(0., 0., 0.), cv::Point3d(lenght*( (m_dataFrameBuffer.end() - 1)->undistortedPointsPrev[UNDISTORTED_POINT_NBR](0) ),
				lenght*( (m_dataFrameBuffer.end() - 1)->undistortedPointsPrev[UNDISTORTED_POINT_NBR](1) ),lenght),
				cv::viz::Color::green());
		cv::viz::WLine line2(cv::Point3d(0., 0., 0.), cv::Point3d(lenght*( (m_dataFrameBuffer.end() - 1)->undistortedPointsCurr[UNDISTORTED_POINT_NBR](0) ),
				lenght*( (m_dataFrameBuffer.end() - 1)->undistortedPointsCurr[UNDISTORTED_POINT_NBR](1) ), lenght),
				cv::viz::Color::green());

		// the reconstructed cloud of 3D points
		cv::viz::WCloud cloud((m_dataFrameBuffer.end() - 1)->points3D, cv::viz::Color::blue());
		cloud.setRenderingProperty(cv::viz::POINT_SIZE, 3.);

		// Add the virtual objects to the environment
		m_visualizer.showWidget("Camera1", cam1);
		m_visualizer.showWidget("Camera2", cam2);
		m_visualizer.showWidget("Cloud", cloud);
		m_visualizer.showWidget("Line1", line1);
		m_visualizer.showWidget("Line2", line2);
		m_visualizer.showWidget("Triangulated", point3D);

		// Move the second camera
		cv::Affine3d pose((m_dataFrameBuffer.end() - 1)->rotationMatrix, (m_dataFrameBuffer.end() - 1)->translationVector);
		m_visualizer.setWidgetPose("Camera2", pose);
		m_visualizer.setWidgetPose("Line2", pose);


		m_visualizer.spinOnce(1,     // pause 1ms
							true); // redraw

		//m_visualizer.spin();

	}
}
