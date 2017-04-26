/*
 * KalmanFilter.cpp
 *
 *  Created on: Apr 19, 2017
 *      Author: Istvan Balazs Opra
 */

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(MatrixXd&& A, MatrixXd&& B, MatrixXd&& Rt) :
	A(std::move(A)),
	B(std::move(B)),
	Rt(std::move(Rt))
{
}
