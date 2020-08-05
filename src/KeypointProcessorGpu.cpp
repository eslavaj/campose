/*
 * KeypointProcessorGpu.cpp
 *
 *  Created on: Jul 29, 2020
 *      Author: jeslava
 */

#include <vector>
#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/calib3d.hpp>

#include "KeypointProcessorGpu.hpp"

using namespace std;

ExtractReturnCode::ExtractReturnCode KeypointProcessorGpu::extractKpointDescriptors(cv::Mat & newImage)
{
	/*To measure time*/
	double t;
	DataFrame frame;
	/*Convert to grayscale*/
    cv::Mat imgGray;
    cv::cvtColor(newImage, frame.cameraImg, cv::COLOR_BGR2GRAY);

    //frame.cameraImg = imgGray;
    frame.gpu_cameraImg.upload(frame.cameraImg);
    //m_dataFrameBuffer.push_back(frame);

    cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

    vector<cv::KeyPoint> keypoints; // create empty feature list for current image
    cv::Mat descriptors; // create empty feature list for current image
    cv::cuda::GpuMat gkeypoints; // this holds the keys detected
    cv::cuda::GpuMat gdescriptors; // this holds the descriptors for the detected keypoints
    cv::cuda::GpuMat mask1; // this holds any mask you may want to use, or can be replace by noArray() in the call below if no mask is needed
    cv::cuda::Stream istream;

    if(m_detectorType.compare("ORB"))
    {
    	/*ORB*/
        cv::Ptr<cv::cuda::ORB> orb = cv::cuda::ORB::create(250, 1.2f, 8, 31, 0, 2, 0, 31, 20, true);
        t = (double)cv::getTickCount();
        orb->detectAndComputeAsync(frame.gpu_cameraImg, mask1, gkeypoints, gdescriptors, false, istream);
        istream.waitForCompletion();
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        orb->convert(gkeypoints, keypoints);
        cout << "#2 : DETECT KEYPOINTS and DESCRIPTORS done " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    }
    else
    {
    	/*FAST*/
    	int threshold = 30;
    	cv::Ptr<cv::cuda::FastFeatureDetector> fastdetector = cv::cuda::FastFeatureDetector::create(threshold, true, cv::FastFeatureDetector::TYPE_9_16);
    	t = (double)cv::getTickCount();
    	fastdetector->detectAndComputeAsync(frame.gpu_cameraImg, mask1, gkeypoints, gdescriptors, false, istream);
    	istream.waitForCompletion();
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    	fastdetector->convert(gkeypoints, keypoints);
    	cout << "#2 : DETECT KEYPOINTS and DESCRIPTORS done " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    }

    gdescriptors.download(descriptors);

    /*Don't use this frame if it has not enough keypoints*/
    if(keypoints.size() < 35)
    {
    	return ExtractReturnCode::NOT_ENOUGH_KEYPOINTS;
    }

    // push keypoints and descriptor for current frame to end of data buffer
    //(m_dataFrameBuffer.end() - 1)->keypoints = keypoints;
    frame.keypoints = keypoints;
    frame.gpu_keypoints = gkeypoints.clone();

    // push descriptors for current frame to end of data buffer
    frame.gpu_descriptors = gdescriptors.clone();
    frame.descriptors = descriptors.clone();

    m_dataFrameBuffer.push_back(frame);

    return ExtractReturnCode::OK;
}


void KeypointProcessorGpu::matchKpoints(string mpointStrategy)
{
	/*To measure time*/
	double t;
	cv::cuda::Stream istream;
	vector< cv::DMatch > matches;

    if (m_dataFrameBuffer.size() > 1) // wait until at least two images have been processed
    {
        cv::Ptr<cv::cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
        //cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);

        cv::cuda::GpuMat gmatches;

        t = (double)cv::getTickCount();

        /* MATCH KEYPOINT DESCRIPTORS */
        if (m_selectorType.compare("SEL_KNN") == 0)
        {
        	cv::cuda::GpuMat gknnMatches;
        	vector<vector<cv::DMatch>> knnMatches;
            matcher->knnMatchAsync( (m_dataFrameBuffer.end() - 2)->gpu_descriptors, (m_dataFrameBuffer.end() - 1)->gpu_descriptors, gknnMatches, 2, cv::noArray(), istream);
        	istream.waitForCompletion();
        	matcher->knnMatchConvert(gknnMatches, knnMatches);

        	double minDescDistRatio = 0.8;
        	for(auto it = knnMatches.begin(); it!=knnMatches.end(); it++)
        	{
        		if((*it)[0].distance < minDescDistRatio*( (*it)[1].distance ) )
        		{
        			matches.push_back((*it)[0]);
        		}
        	}
        }
        else
        {
        	matcher->matchAsync( (m_dataFrameBuffer.end() - 2)->gpu_descriptors, (m_dataFrameBuffer.end() - 1)->gpu_descriptors, gmatches, cv::noArray(), istream);
        	istream.waitForCompletion();
        	matcher->matchConvert(gmatches, matches);
        }

        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "#3 : MATCH KEYPOINT DESCRIPTORS done " << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;

    	if(matches.size() < 8)
    	{
    		cout << "################################ There is not enough matches ignore this frame ################################" << endl;
    		m_dataFrameBuffer.clear();
    		return;
    	}

        if(mpointStrategy.compare("NONE") != 0)
        {
        	vector< cv::DMatch > ransacCorrtedMatches;
        	t = (double)cv::getTickCount();
            /*Do RANSAC test*/
            if(refineMatches(matches, (m_dataFrameBuffer.end() - 2)->keypoints, (m_dataFrameBuffer.end() - 1)->keypoints, ransacCorrtedMatches, mpointStrategy) !=
            		RefineReturnCode::OK)
            {
            	m_dataFrameBuffer.clear();
            	return;
            }
            t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
            cout << "#4 : REFINE MATCHES DONE " << ransacCorrtedMatches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
            matches = ransacCorrtedMatches;
        }
        else
        {
        	/*TODO*/
        }

        // store matches in current data frame
        (m_dataFrameBuffer.end() - 1)->kptMatches = matches;
        (m_dataFrameBuffer.end() - 1)->gpu_kptMatches = gmatches.clone();

        // visualize matches between current and previous image
        /*
        if (m_visuEnable)
        {
            cv::Mat matchImg = ((m_dataFrameBuffer.end() - 1)->cameraImg).clone();

            cv::drawMatches((m_dataFrameBuffer.end() - 2)->cameraImg, (m_dataFrameBuffer.end() - 2)->keypoints,
                            (m_dataFrameBuffer.end() - 1)->cameraImg, (m_dataFrameBuffer.end() - 1)->keypoints,
                            matches, matchImg,
                            cv::Scalar::all(-1), cv::Scalar::all(-1),
                            vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            string windowName = "Matching keypoints between two camera images";
            cv::namedWindow(windowName, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
            cv::imshow(windowName, matchImg);
            cout << "Press key to continue to next image" << endl;
            //cv::waitKey(0); // wait for key to be pressed
            usleep(100000);
        }
        */
    }
}

// Identify good matches using RANSAC
// Return fundamental matrix and output matches
RefineReturnCode::RefineReturnCode KeypointProcessorGpu::refineMatches(const std::vector<cv::DMatch>& matches,
	                 std::vector<cv::KeyPoint>& keypoints1,
					 std::vector<cv::KeyPoint>& keypoints2,
				     std::vector<cv::DMatch>& outMatches, string matchRefineStrategy) {

	// Convert keypoints into Point2f
	std::vector<cv::Point2f> points1, points2;
	cv::Mat outMatrix;

	for (std::vector<cv::DMatch>::const_iterator it= matches.begin(); it!= matches.end(); it++)
	{
		if(it->queryIdx<0)
		{
			cout<< "it->queryIdx: "<< it->queryIdx<<endl;
			cout<< "it->trainIdx: "<< it->trainIdx<<endl;
		}

		// Get the position of left keypoints
		points1.push_back(keypoints1[it->queryIdx].pt);
		// Get the position of right keypoints
		points2.push_back(keypoints2[it->trainIdx].pt);
	}

	// Compute F matrix using RANSAC
	std::vector<uchar> inliers(points1.size(),0);

	if(matchRefineStrategy.compare("FUND")==0)
	{

		outMatrix= cv::findFundamentalMat(
				points1,points2, // matching points
				inliers,         // match status (inlier or outlier)
				cv::FM_RANSAC,   // RANSAC method
				m_distToEpipLine,        // distance to epipolar line
				m_ransacConfid);     // confidence probability

	}
	else if (matchRefineStrategy.compare("HOMOGR")==0)
	{
		outMatrix = cv::findHomography(
				points1,points2, // corresponding points
				inliers,	     // outputed inliers matches
				cv::RANSAC,	     // RANSAC method
				m_distToEpipLine);// max distance to reprojection point
	}
	else
	{
		/*Other cases TODO*/
	}

	// extract the surviving (inliers) matches
	std::vector<uchar>::const_iterator itIn= inliers.begin();
	std::vector<cv::DMatch>::const_iterator itM= matches.begin();
	// for all matches
	for ( ;itIn!= inliers.end(); ++itIn, ++itM)
	{
		if (*itIn)
		{ // it is a valid match
			outMatches.push_back(*itM);
		}
	}

	/*Check if there is enough inliers to recalculate Fundamental Matrix*/
	if(outMatches.size() < 8)
	{
		return RefineReturnCode::NOT_ENOUGH_INLIERS;
	}

	if(matchRefineStrategy.compare("FUND")==0)
	{
		if (m_refineFund || m_refineMatches)
		{
			// The F matrix will be recomputed with all accepted matches
			// Convert keypoints into Point2f for final F computation
			points1.clear();
			points2.clear();

			for (std::vector<cv::DMatch>::const_iterator it= outMatches.begin(); it!= outMatches.end(); ++it)
			{
				// Get the position of left keypoints
				points1.push_back(keypoints1[it->queryIdx].pt);
				// Get the position of right keypoints
				points2.push_back(keypoints2[it->trainIdx].pt);
			}

			// Compute 8-point F from all accepted matches
			outMatrix= cv::findFundamentalMat(
					points1,points2, // matching points
					cv::FM_8POINT); // 8-point method

			if (m_refineMatches) {

				std::vector<cv::Point2f> newPoints1, newPoints2;
				// refine the matches

				correctMatches(outMatrix,             // F matrix
						points1, points2,        // original position
						newPoints1, newPoints2); // new position

				for (int i=0; i< points1.size(); i++)
				{
					/*
					std::cout << "(" << keypoints1[outMatches[i].queryIdx].pt.x
						      << "," << keypoints1[outMatches[i].queryIdx].pt.y
							  << ") -> ";
					std::cout << "(" << newPoints1[i].x
						      << "," << newPoints1[i].y << std::endl;
					std::cout << "(" << keypoints2[outMatches[i].trainIdx].pt.x
						      << "," << keypoints2[outMatches[i].trainIdx].pt.y
							  << ") -> ";
					std::cout << "(" << newPoints2[i].x
						      << "," << newPoints2[i].y <<")"<< std::endl;
					 */
					keypoints1[outMatches[i].queryIdx].pt.x= newPoints1[i].x;
					keypoints1[outMatches[i].queryIdx].pt.y= newPoints1[i].y;
					keypoints2[outMatches[i].trainIdx].pt.x= newPoints2[i].x;
					keypoints2[outMatches[i].trainIdx].pt.y= newPoints2[i].y;
				}
				(m_dataFrameBuffer.end() - 1)->refinedPointsPrev = newPoints1;
				(m_dataFrameBuffer.end() - 1)->refinedPointsCurr = newPoints2;
			}

			m_fundMatrix = outMatrix.clone();
		}
	}
	else if(matchRefineStrategy.compare("HOMOGR")==0)
	{
		/*HOMOGRAPHY */
		m_homographyMatrix = outMatrix.clone();
	}
	else
	{
		/*Other cases TODO*/
	}

	return RefineReturnCode::OK;
}


void KeypointProcessorGpu::visualize(double wait_uSec)
{
    if (m_dataFrameBuffer.size() > 1) // wait until at least two images have been processed
    {
        // visualize matches between current and previous image
        if (m_visuEnable)
        {
            cv::Mat matchImg = ((m_dataFrameBuffer.end() - 1)->cameraImg).clone();

            cv::drawMatches((m_dataFrameBuffer.end() - 2)->cameraImg, (m_dataFrameBuffer.end() - 2)->keypoints,
                            (m_dataFrameBuffer.end() - 1)->cameraImg, (m_dataFrameBuffer.end() - 1)->keypoints,
    						(m_dataFrameBuffer.end() - 1)->kptMatches, matchImg,
                            cv::Scalar::all(-1), cv::Scalar::all(-1),
                            vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            string windowName = "Matching keypoints between two camera images";
            cv::namedWindow(windowName, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
            cv::imshow(windowName, matchImg);
            cout << "Press key to continue to next image" << endl;
            //cv::waitKey(0); // wait for key to be pressed
            usleep(wait_uSec);
        }
    }
}
