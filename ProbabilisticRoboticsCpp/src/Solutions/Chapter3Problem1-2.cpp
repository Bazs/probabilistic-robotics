/*
 * Chapter3Problem1.cpp
 *
 *  Created on: Apr 22, 2017
 *      Author: root
 */
#include <iostream>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "CommonTypes.h"
#include "KalmanFilter.h"

using std::move;

USIGN32 const numStates = 2UL;
USIGN32 const numControls = 1UL;
USIGN32 const numMeasurements = 1UL;
using Filter = KalmanFilter<numStates, numControls, numMeasurements, Eigen::FullPivHouseholderQR<Eigen::Matrix<double, numMeasurements, numMeasurements>>>;

using SMat = Filter::SMat;
using CMat = Filter::CMat;
using SMMat = Filter::SMMat;
using MMat = Filter::MMat;
using SVec = Filter::SVec;
using CVec = Filter::CVec;
using MVec = Filter::MVec;

int main()
{
	SMat A;
	A << 1, 1, 0, 1;
	CMat B;
	B << 0, 0;
	SMat R;
	R << 0.25, 0.5, 0.5, 1;
	SMMat C;
	C << 1, 0;
	MMat Q;
	Q << 10;

	Filter filter(move(A), move(B), move(C), move(R), move(Q));

	SVec initialMu;
	initialMu << 0, 0;
	SMat initialSigma;
	initialSigma << 0, 0, 0, 0;

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

	measurement << 5;
	filter.update(controls, measurement);
	std::cout << "Measurement: " << measurement << ", result:" << std::endl << filter;
}

