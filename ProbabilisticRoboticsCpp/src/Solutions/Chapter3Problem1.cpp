/*
 * Chapter3Problem1.cpp
 *
 *  Created on: Apr 22, 2017
 *      Author: root
 */
#include "eigen3/Eigen/Core"
#include "CommonTypes.h"
#include "KalmanFilter.h"

using std::move;

USIGN32 const numStates = 2UL;
using FMat = KalmanFilter<numStates>::FMat;
using FVec = KalmanFilter<numStates>::FVec;

int main()
{
	FMat A;
	A << 1, 1, 0, 1;

	FMat B;

	FMat Rt;
	Rt << 0.25, 0.5, 0.5, 1;

	KalmanFilter<numStates> filter(move(A), move(B), move(Rt));
}

