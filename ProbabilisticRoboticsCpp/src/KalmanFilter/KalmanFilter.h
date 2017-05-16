/*
 * KalmanFilter.h
 *
 *  Created on: Apr 19, 2017
 *      Author: Istvan Balazs Opra
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <KalmanFilterBase.h>

template <USIGN32 numStates, USIGN32 numControls, USIGN32 numMeasurements,
	template<class> class Decomposition = Eigen::ColPivHouseholderQR>
class KalmanFilter : public KalmanFilterBase<numStates, numControls, numMeasurements, Decomposition>
{
public:
	using Base = KalmanFilterBase<numStates, numControls, numMeasurements, Decomposition>;
	using SMat = typename Base::SMat;
	using CMat = typename Base::CMat;
	using SMMat = typename Base::SMMat;
	using MMat = typename Base::MMat;
	using SVec = typename Base::SVec;
	using CVec = typename Base::CVec;
	using MVec = typename Base::MVec;

	KalmanFilter(SMat&& A, CMat&& B, SMMat&& C, SMat&& R, MMat&& Q) :
		Base(std::forward<SMat>(R), std::forward<MMat>(Q)),
		A(std::forward<SMat>(A)),
		B(std::forward<CMat>(B)),
		C(std::forward<SMMat>(C))
	{}

	KalmanFilter(const KalmanFilter& other) = delete;
	KalmanFilter& operator=(const KalmanFilter& other) = delete;

	virtual ~KalmanFilter()
	{}

	virtual void update(CVec& controls, MVec& measurements)
	{
		predictionStep(controls);
		measurementUpdate(measurements);
	}
private:
	SMat A;
	CMat B;
	SMMat C;

	void predictionStep(CVec& controls)
	{
		Base::mu = A * Base::mu + B * controls;
		Base::sigma = A * Base::sigma * A.transpose() + Base::R;
	}

	void measurementUpdate(MVec& measurements)
	{
		const SVec& K = Base::calculateKalmanGain(C);
		Base::mu = Base::mu + K * (measurements - C * Base::mu);
		Base::sigma = (SMat::Identity() - K * C) * Base::sigma;
	}
};

#endif /* KALMAN_FILTER_H */
