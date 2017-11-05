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

	using SFun = std::function<void(const CVec&, SVec&)>;
	using GFun = std::function<SMat(const CVec&, SVec&)>;
	using MFun = std::function<CVec(const SVec&)>;
	using HFun = std::function<SMMat(const SVec&)>;

	ExtendedKalmanFilter(SFun&& g, GFun&& gJacobian, MFun&& h, HFun&& hJacobian, SMat&& R, MMat&& Q) :
		Base(std::forward<SMat>(R), std::forward<MMat>(Q)),
		g(g),
		gFun(gJacobian),
		h(h),
		hFun(hJacobian)
	{}

	virtual ~ExtendedKalmanFilter()
	{}

	virtual void update(CVec& controls, MVec& measurements)
	{
		predictionStep(controls);
		measurementUpdate(measurements);
	}

	void predictionStep(CVec& controls)
	{
		SMat G = gFun(controls, Base::mu);
		Base::sigma = G * Base::sigma * G.transpose() + Base::R;
		g(controls, Base::mu);
	}

	void measurementUpdate(MVec& measurements)
	{
		SMMat H = hFun(Base::mu);
		const SVec& K = Base::calculateKalmanGain(H);
		Base::mu = Base::mu + K * (measurements - h(Base::mu));
		Base::sigma = (SMat::Identity() - K * H) * Base::sigma;
	}

private:
	SFun g;
	GFun gFun;
	MFun h;
	HFun hFun;
};

#endif // EXTENDED_KALMAN_FILTER_H
