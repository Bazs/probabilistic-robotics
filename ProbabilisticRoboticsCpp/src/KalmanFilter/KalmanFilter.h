/*
 * KalmanFilter.h
 *
 *  Created on: Apr 19, 2017
 *      Author: Istvan Balazs Opra
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <eigen3/Eigen/Core>

using Eigen::MatrixXd;

class KalmanFilter
{
public:
	KalmanFilter(MatrixXd &&A, MatrixXd &&B, MatrixXd &&Rt);
private:
	MatrixXd A;
	MatrixXd B;
	MatrixXd Rt;
};

#endif /* KALMAN_FILTER_H */
