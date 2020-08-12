/*
 * Gtsamer.hpp
 *
 *  Created on: Aug 11, 2020
 *      Author: jeslava
 */

#ifndef GTSAMER_HPP_
#define GTSAMER_HPP_

#include "gtsam/geometry/Pose3.h"
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/circular_buffer.hpp>
#include "dataStructures.h"

class Gtsamer
{

	Gtsamer(boost::circular_buffer<DataFrame> & dataBuffer);
	void addLastFrameToGraph();


private:
	boost::circular_buffer<DataFrame> & m_dataFrameBuffer;
	gtsam::NonlinearFactorGraph m_graph;
	boost::shared_ptr m_sigmas;

};




class UnaryFactor: public gtsam::NoiseModelFactor1<gtsam::Pose3> {
  // The factor will hold a measurement consisting of an (X,Y) location
  // We could this with a Point2 but here we just use two doubles
  double mx_, my_, mz_;

 public:
  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<UnaryFactor> shared_ptr;

  // The constructor requires the variable key, the (X, Y) measurement value, and the noise model
  UnaryFactor(gtsam::Key j, double x, double y, double z, const gtsam::SharedNoiseModel& model):
    NoiseModelFactor1<gtsam::Pose3>(model, j), mx_(x), my_(y), mz_(z) {}

  virtual ~UnaryFactor() {}

  // Using the NoiseModelFactor1 base class there are two functions that must be overridden.
  // The first is the 'evaluateError' function. This function implements the desired measurement
  // function, returning a vector of errors when evaluated at the provided variable value. It
  // must also calculate the Jacobians for this measurement function, if requested.
  gtsam::Vector evaluateError(const gtsam::Pose3& q,
                       boost::optional<gtsam::Matrix&> H = boost::none) const {
    // The measurement function for a GPS-like measurement is simple:
    // error_x = pose.x - measurement.x
    // error_y = pose.y - measurement.y
    // Consequently, the Jacobians are:
    // [ derror_x/dx  derror_x/dy  derror_x/dz ] = [1 0 0]
    // [ derror_y/dx  derror_y/dy  derror_y/dz ] = [0 1 0]
	// [ derror_z/dx  derror_z/dy  derror_z/dz ] = [0 0 1]
    if (H) (*H) = (gtsam::Matrix(2, 3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0).finished();
    return (gtsam::Vector(3) << q.x() - mx_, q.y() - my_, q.z() - mz_).finished();
  }

  // The second is a 'clone' function that allows the factor to be copied. Under most
  // circumstances, the following code that employs the default copy constructor should
  // work fine.
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this))); }

  // Additionally, we encourage you the use of unit testing your custom factors,
  // (as all GTSAM factors are), in which you would need an equals and print, to satisfy the
  // GTSAM_CONCEPT_TESTABLE_INST(T) defined in Testable.h, but these are not needed below.
};  // UnaryFactor





#endif /* GTSAMER_HPP_ */
