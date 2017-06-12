/*
 * Chapter3Problem1.cpp
 *
 *  Created on: Apr 22, 2017
 *      Author: root
 */
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <CommonTypes.h>
#include <ExtendedKalmanFilter.h>
#include <cmath>

using std::move;

USIGN32 const numStates = 3UL;
USIGN32 const numControls = 1UL;
USIGN32 const numMeasurements = 1UL;
using Filter = ExtendedKalmanFilter<numStates, numControls, numMeasurements, Eigen::FullPivHouseholderQR>;

using SMat = Filter::SMat;
using CMat = Filter::CMat;
using SMMat = Filter::SMMat;
using MMat = Filter::MMat;
using SVec = Filter::SVec;
using CVec = Filter::CVec;
using MVec = Filter::MVec;

void stateTransitionFun(const CVec& controls, SVec& mu)
{
	double x = mu(0);
	double y = mu(1);
	double th = mu(2);
	double d = controls(0);
	mu(0) = x + cos(th) * d;
	mu(1) = y + sin(th) * d;
}

CVec measurementFun(const SVec& mu)
{
	return CVec();
}

int main()
{
	SMat G = SMat::Identity();
	SMat R = SMat::Zero();
	R << 0.25, 0.5, 0.5, 1;
	SMMat H = SMMat::Identity();
	MMat Q = MMat::Identity();

	Filter filter(stateTransitionFun, move(G), measurementFun, move(H), move(R), move(Q));

	SVec initialMu;
	initialMu << 0, 0, 0;
	SMat initialSigma;
	initialSigma << 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 10000.0;

	filter.setBelief(move(initialMu), move(initialSigma));

	CVec controls;
	controls << 0;

	MVec measurement;
	measurement << 0;

	for (int iteration = 0; iteration < 4; ++iteration)
	{
		filter.update(controls, measurement);
		std::cout << "Iteration " << iteration + 1 << ":" << std::endl << filter << std::endl << std::endl;
	}
}

