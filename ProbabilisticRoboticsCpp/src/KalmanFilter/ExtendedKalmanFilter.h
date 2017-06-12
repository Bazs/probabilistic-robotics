/*
 * ExtendedKalmanFilter.h
 *
 *  Created on: May 7, 2017
 *      Author: Istvan Balazs Opra
 */
#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include <KalmanFilterBase.h>

template <USIGN32 numStates, USIGN32 numControls, USIGN32 numMeasurements,
	template<class> class Decomposition = Eigen::ColPivHouseholderQR>
class ExtendedKalmanFilter : public KalmanFilterBase<numStates, numControls, numMeasurements, Decomposition>
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

	using SFun = std::function<void(const CVec&,SVec&)>;
	using MFun = std::function<CVec(const SVec&)>;

	ExtendedKalmanFilter(SFun g, SMat&& G, MFun h, SMMat&& H, SMat&& R, MMat&& Q) :
		Base(std::forward<SMat>(R), std::forward<MMat>(Q)),
		g(g),
		G(std::forward<SMat>(G)),
		h(h),
		H(std::forward<SMMat>(H))
	{}

	virtual ~ExtendedKalmanFilter()
	{}

	virtual void update(CVec& controls, MVec& measurements)
	{
		predictionStep(controls);
		measurementUpdate(measurements);
	}

private:
	SFun g;
	SMat G;
	MFun h;
	SMMat H;

	void predictionStep(CVec& controls)
	{
		g(controls, Base::mu);
		Base::sigma = G * Base::sigma * G.transpose() + Base::R;
	}

	void measurementUpdate(MVec& measurements)
	{
		const SVec& K = Base::calculateKalmanGain(H);
		Base::mu = Base::mu + K * (measurements - h(Base::mu));
		Base::sigma = (SMat::Identity() - K * H) * Base::sigma;
	}
};

#endif // EXTENDED_KALMAN_FILTER_H
