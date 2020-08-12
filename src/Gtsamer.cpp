/*
 * GtSamer.cpp
 *
 *  Created on: Aug 11, 2020
 *      Author: jeslava
 */

#include "Gtsamer.hpp"
#include "dataStructures.h"

#include <boost/circular_buffer.hpp>


// We will use Pose2 variables (x, y, theta) to represent the robot positions
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

// We will use simple integer Keys to refer to the robot poses.
#include <gtsam/inference/Key.h>

// As in OdometryExample.cpp, we use a BetweenFactor to model odometry measurements.
#include <gtsam/slam/BetweenFactor.h>

// We add all facors to a Nonlinear Factor Graph, as our factors are nonlinear.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// standard Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

using namespace std;
using namespace gtsam;


// Before we begin the example, we must create a custom unary factor to implement a
// "GPS-like" functionality. Because standard GPS measurements provide information
// only on the position, and not on the orientation, we cannot use a simple prior to
// properly model this measurement.
//
// The factor will be a unary factor, affect only a single system variable. It will
// also use a standard Gaussian noise model. Hence, we will derive our new factor from
// the NoiseModelFactor1.
#include <gtsam/nonlinear/NonlinearFactor.h>



Gtsamer::Gtsamer(boost::circular_buffer<DataFrame> & dataBuffer)
{

	m_dataFrameBuffer = dataBuffer;

	// 1. Create a factor graph container and add factors to it
	//NonlinearFactorGraph graph;

	// 2a. Add odometry factors
	// For simplicity, we will use the same noise model for each odometry factor
	//auto odometryNoise = noiseModel::Diagonal::Sigmas((Vector(6)<<0.4,0.4,0.4,0.5,0.5,0.5).finished());
	m_sigmas = noiseModel::Diagonal::Sigmas((Vector(6)<<0.4,0.4,0.4,0.5,0.5,0.5).finished());
	// Create odometry (Between) factors between consecutive poses


}


void Gtsamer::addLastFrameToGraph()
{

	if(m_dataFrameBuffer.size() >1)
	{

		cv::Mat rotation_tmp = (m_dataFrameBuffer.end() - 1)->rotationMatrix;
		cv::Mat translation_tmp = (m_dataFrameBuffer.end() - 1)->translationVector;

		Rot3 R(rotation_tmp.at<double>(0,0), rotation_tmp.at<double>(0,1), rotation_tmp.at<double>(0,2),
				rotation_tmp.at<double>(1,0), rotation_tmp.at<double>(1,1), rotation_tmp.at<double>(1,2),
				rotation_tmp.at<double>(2,0), rotation_tmp.at<double>(2,1), rotation_tmp.at<double>(2,2));
		Point3 t(translation_tmp.at<double>(0,0),translation_tmp.at<double>(1,0), translation_tmp.at<double>(2,0));

		(m_dataFrameBuffer.end() - 1)->gtsamRelPose = Pose3(R, t);
		int graphPrevNode_tmp = (m_dataFrameBuffer.end() - 2)->graphPrevNode + 1;
		(m_dataFrameBuffer.end() - 1)->graphPrevNode = graphPrevNode_tmp;

		//graph.emplace_shared<BetweenFactor<Pose2> >(1, 2, Pose2(2.0, 0.0, 0.0), odometryNoise);
		m_graph.emplace_shared<BetweenFactor<Pose3> >(graphPrevNode_tmp, graphPrevNode_tmp + 1, Pose3(R, t), m_sigmas);


		// 2b. Add "GPS-like" measurements
		// We will use our custom UnaryFactor for this.
		auto unaryNoise =  noiseModel::Diagonal::Sigmas((Vector(6)<<0.4,0.4,0.4,0.5,0.5,0.5).finished());  // 10cm std on x,y
	    /*TODO: Simulate GPS data*/
		m_graph.emplace_shared<UnaryFactor>(1, estimatedX, estimatedY, estimatedZ, unaryNoise);

		// 3. Create the data structure to hold the initialEstimate estimate to the solution
		// For illustrative purposes, these have been deliberately set to incorrect values
		Values initialEstimate;
		cv::Affine3d::Mat3 R_tmp = (m_dataFrameBuffer.end() - 1)->pose.rotation();
		cv::Affine3d::Vec3 t_tmp = (m_dataFrameBuffer.end() - 1)->pose.translation();

		Rot3 RinitEst(R_tmp.val[0], R_tmp.val[1], R_tmp.val[2],
					  R_tmp.val[3], R_tmp.val[4], R_tmp.val[5],
					  R_tmp.val[6], R_tmp.val[7], R_tmp.val[8]);
		Point3 tinitEst(t_tmp.val[0], t_tmp.val[1], t_tmp.val[2]);
		initialEstimate.insert(graphPrevNode_tmp, Pose3(RinitEst, tinitEst));
		initialEstimate.print("\nInitial Estimate:\n");

	}

}
