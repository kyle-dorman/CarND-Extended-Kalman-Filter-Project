#include "kalman_filter.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Build empty KalmanFilter
 */
KalmanFilter KalmanFilter::Empty() {
	VectorXd x = VectorXd(4);
	MatrixXd P = MatrixXd(4, 4);
	KalmanFilter k = KalmanFilter(x, P, 0);
	k.initialized_ = false;
	return k;
}

/*
 * Constructor.
 */
KalmanFilter::KalmanFilter(const VectorXd &x_in_, const MatrixXd &P_in_, long timestamp) {
  initialized_ = true;
  timestamp_ = timestamp;
  P_ = P_in_;
  x_ = x_in_;
}

/**
* Destructor.
*/
KalmanFilter::~KalmanFilter() {}
