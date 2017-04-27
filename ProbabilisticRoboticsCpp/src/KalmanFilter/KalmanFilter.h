/*
 * KalmanFilter.h
 *
 *  Created on: Apr 19, 2017
 *      Author: Istvan Balazs Opra
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <eigen3/Eigen/Core>

template <unsigned int numStates>
class KalmanFilter
{
public:
	typedef Eigen::Matrix<double, numStates, numStates> FMat;
	typedef Eigen::Matrix<double, 1, numStates> FVec;

	KalmanFilter(FMat &&A, FMat &&B, FMat &&Rt) :
		A(std::move(A)),
		B(std::move(B)),
		Rt(std::move(Rt))
	{}

	void setBelief(FVec &&mu, FMat &&Sigma)
	{
		this->mu = mu;
		this->Sigma = Sigma;
	}

	void predictionStep()
	{

	}
private:
	FMat A;
	FMat B;
	FMat Rt;

	FVec mu;
	FMat Sigma;
};

#endif /* KALMAN_FILTER_H */
