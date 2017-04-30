/*
 * KalmanFilter.h
 *
 *  Created on: Apr 19, 2017
 *      Author: Istvan Balazs Opra
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

template <USIGN32 numStates, USIGN32 numControls, USIGN32 numMeasurements,
	typename Decomposition = Eigen::ColPivHouseholderQR<Eigen::Matrix<double, numMeasurements, numMeasurements>>>
class KalmanFilter
{
public:
	typedef Eigen::Matrix<double, numStates, numStates> SMat;
	typedef Eigen::Matrix<double, numStates, numControls> CMat;
	typedef Eigen::Matrix<double, numMeasurements, numStates> SMMat;
	typedef Eigen::Matrix<double, numMeasurements, numMeasurements> MMat;
	typedef Eigen::Matrix<double, numStates, 1> SVec;
	typedef Eigen::Matrix<double, numControls, 1> CVec;
	typedef Eigen::Matrix<double, numMeasurements, 1> MVec;

	KalmanFilter(SMat &&A, CMat &&B, SMMat &&C, SMat &&Rt, MMat &&Q) :
		A(std::move(A)),
		B(std::move(B)),
		C(std::move(C)),
		R(std::move(Rt)),
		Q(std::move(Q))
	{}

	void setBelief(SVec &&mu, SMat &&Sigma)
	{
		this->mu = mu;
		this->sigma = Sigma;
	}

	void update (CVec& controls, MVec& measurements)
	{
		predictionStep(controls);
		measurementUpdate(measurements);
	}

	template<USIGN32 nStates, USIGN32 nControls, USIGN32 nMeasurements, typename Decomp>
	friend std::ostream& operator<<(std::ostream& os, const KalmanFilter<nStates, nControls, nMeasurements, Decomp>&  filter);
private:
	SMat A;
	CMat B;
	SMMat C;
	SMat R;
	MMat Q;

	SVec mu;
	SMat sigma;

	void predictionStep(CVec& controls)
	{
		mu = A * mu + B * controls;
		sigma = A * sigma * A.transpose() + R;
	}

	void measurementUpdate(MVec& measurements)
	{
		SVec kGain = sigma * C.transpose() * Decomposition(C * sigma * C.transpose() + Q).inverse();
		mu = mu + kGain * (measurements - C * mu);
		sigma = (SMat::Identity() - kGain * C) * sigma;
	}
};

template <USIGN32 numStates, USIGN32 numControls, USIGN32 numMeasurements, typename Decomposition>
std::ostream& operator<<(std::ostream& os, const KalmanFilter<numStates, numControls, numMeasurements, Decomposition>& filter)
{
	os << "mu: " << std::endl << filter.mu << std::endl << "Sigma: " << std::endl << filter.sigma;
	return os;
}


#endif /* KALMAN_FILTER_H */
