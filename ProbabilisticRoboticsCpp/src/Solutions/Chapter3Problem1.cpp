/*
 * Chapter3Problem1.cpp
 *
 *  Created on: Apr 22, 2017
 *      Author: root
 */
#include <iostream>
#include "eigen3/Eigen/Core"
#include "CommonTypes.h"
#include "KalmanFilter.h"

using std::move;

USIGN32 const numStates = 2UL;
USIGN32 const numControls = 1UL;
using Filter = KalmanFilter<numStates, numControls>;

using SMat = Filter::SMat;
using CMat = Filter::CMat;
using SVec = Filter::SVec;
using CVec = Filter::CVec;

int main()
{
	SMat A;
	A << 1, 1, 0, 1;
	CMat B;
	B << 0, 0;
	SMat Rt;
	Rt << 0.25, 0.5, 0.5, 1;

	Filter filter(move(A), move(B), move(Rt));

	SVec initialMu;
	initialMu << 0, 0;
	SMat initialSigma;
	initialSigma << 0, 0, 0, 0;

	filter.setBelief(move(initialMu), move(initialSigma));

	CVec controls;
	controls << 0;

	for (int iteration = 0; iteration < 5; ++iteration)
	{
		filter.update(controls);
		std::cout << "Iteration " << iteration + 1 << ":" << std::endl << filter << std::endl;
	}
}

