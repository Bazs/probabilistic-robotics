/*
 * KalmanFilterBase.h
 *
 *  Created on: May 7, 2017
 *      Author: Istvan Balazs Opra
 */
#ifndef KALMAN_FILTER_BASE_H
#define KALMAN_FILTER_BASE_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

template <USIGN32 numStates, USIGN32 numControls, USIGN32 numMeasurements,
	template<class> class Decomposition = Eigen::ColPivHouseholderQR>
class KalmanFilterBase
{
public:
	using SMat = Eigen::Matrix<double, numStates, numStates>;
	using CMat = Eigen::Matrix<double, numStates, numControls>;
	using SMMat = Eigen::Matrix<double, numMeasurements, numStates>;
	using MMat = Eigen::Matrix<double, numMeasurements, numMeasurements>;
	using SVec = Eigen::Matrix<double, numStates, 1>;
	using CVec = Eigen::Matrix<double, numControls, 1>;
	using MVec = Eigen::Matrix<double, numMeasurements, 1>;

	KalmanFilterBase(SMat&& R, MMat&& Q) : R(std::move(R)), Q(std::move(Q)), mu(), sigma(), decomposition()
	{}

	virtual ~KalmanFilterBase()
	{}

	void setBelief(SVec&& mu, SMat&& sigma)
	{
		this->mu = mu;
		this->sigma = sigma;
	}

	SVec getMean()
	{
		return mu;
	}

	SMat getCovariance()
	{
		return sigma;
	}

	virtual void update(CVec& controls, MVec& measurements) = 0;

	template<USIGN32 nStates, USIGN32 nControls, USIGN32 nMeasurements, template<class> class Decomp>
	friend std::ostream& operator<<(std::ostream& os, const KalmanFilterBase<nStates, nControls, nMeasurements, Decomp>&  filter);
protected:
	SMat R;
	MMat Q;

	SVec mu;
	SMat sigma;
	Decomposition<MMat> decomposition;

	SVec calculateKalmanGain(SMMat& MeasProbMat)
	{
		return sigma * MeasProbMat.transpose()
				* decomposition.compute(MeasProbMat * sigma * MeasProbMat.transpose() + Q).inverse();
	}
};

template <USIGN32 numStates, USIGN32 numControls, USIGN32 numMeasurements, template<class> class Decomposition>
std::ostream& operator<<(std::ostream& os, const KalmanFilterBase<numStates, numControls, numMeasurements, Decomposition>& filter)
{
	os << "mu: " << std::endl << filter.mu << std::endl << "Sigma: " << std::endl << filter.sigma;
	return os;
}

#endif // KALMAN_FILTER_BASE_H
