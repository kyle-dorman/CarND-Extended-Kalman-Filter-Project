#include "FunctionalKalmanFilter.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Constructor.
 */
FunctionalKalmanFilter::FunctionalKalmanFilter() {
	initialized_ = false;
	x_ = VectorXd(4);
	P_ - MatrixXd(4, 4);
	timestamp_ = 0;
}

/*
 * Constructor 2.
 */
FunctionalKalmanFilter::FunctionalKalmanFilter(VectorXd &x_in_, MatrixXd &P_in_, long timestamp) {
	initialized_ = true;
	timestamp_ = timestamp;
	P_ = P_in_;
	x_ = x_in_;
}

/**
* Destructor.
*/
FunctionalKalmanFilter::~FunctionalKalmanFilter() {}
