/*
 * KalmanFilter.h
 *
 *  Created on: Apr 19, 2017
 *      Author: Istvan Balazs Opra
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <eigen3/Eigen/Core>

template <USIGN32 numStates, USIGN32 numControls>
class KalmanFilter
{
public:
	typedef Eigen::Matrix<double, numStates, numStates> SMat;
	typedef Eigen::Matrix<double, numStates, numControls> CMat;
	typedef Eigen::Matrix<double, numStates, 1> SVec;
	typedef Eigen::Matrix<double, numControls, 1> CVec;

	KalmanFilter(SMat &&A, CMat &&B, SMat &&Rt) :
		A(std::move(A)),
		B(std::move(B)),
		Rt(std::move(Rt))
	{}

	void setBelief(SVec &&mu, SMat &&Sigma)
	{
		this->mu = mu;
		this->sigma = Sigma;
	}

	void update (CVec &controls)
	{
		predictionStep(controls);
	}

	template<USIGN32 nStates, USIGN32 nControls>
	friend std::ostream& operator<<(std::ostream& os, const KalmanFilter<nStates, nControls>&  filter);
private:
	SMat A;
	CMat B;
	SMat Rt;

	SVec mu;
	SMat sigma;

	void predictionStep(CVec &controls)
	{
		mu = A * mu + B * controls;
		sigma = A * sigma * A.transpose() + Rt;
	}
};

template <USIGN32 numStates, USIGN32 numControls>
std::ostream& operator<<(std::ostream& os, const KalmanFilter<numStates, numControls>& filter)
{
	os << "mu: " << std::endl << filter.mu << std::endl << "Sigma: " << std::endl << filter.sigma;
	return os;
}


#endif /* KALMAN_FILTER_H */
