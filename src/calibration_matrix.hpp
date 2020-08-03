/*
 * calibration_matrix.hpp
 *
 *  Created on: Aug 1, 2020
 *      Author: jeslava
 */

#ifndef CALIBRATION_MATRIX_HPP_
#define CALIBRATION_MATRIX_HPP_


float calib_elem[9] = { 846.169, 0, 401.32, 0, 843.503, 293.051, 0, 0, 1 };
cv::Mat K = cv::Mat(3, 3, CV_32F, calib_elem);




#endif /* CALIBRATION_MATRIX_HPP_ */
