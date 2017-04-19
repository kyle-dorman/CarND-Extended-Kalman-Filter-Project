#ifndef EKF_HELPER_H_
#define EKF_HELPER_H_

#include "Eigen/Dense"
#include "measurement_package.h"
#include "kalman_filter.h"

namespace ekf_helper {

KalmanFilter Initialize(const MeasurementPackage &measurement_pack);

Eigen::MatrixXd R_laser();

Eigen::MatrixXd R_radar();

Eigen::MatrixXd H_laser();

double TimeChange(long previous_timestamp, long current_timestamp);

Eigen::MatrixXd Q(double time_change, double noise_ax, double noise_ay);

Eigen::MatrixXd F(double time_change);

Eigen::MatrixXd I(int size);

// compute Jacobian Matrix
Eigen::MatrixXd Hj(double px, double py, double vx, double vy);

// Contert cartiesian coordinated to polar
Eigen::VectorXd h(const Eigen::VectorXd &x);

KalmanFilter Predict(KalmanFilter &kalman_filter, const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q);

Eigen::VectorXd KalmanError(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::VectorXd &x);

Eigen::VectorXd ExtendedKalmanError(const Eigen::VectorXd &z, const Eigen::VectorXd &x);

KalmanFilter UpdateKalmanFilterWithError(KalmanFilter const &kalman_filter, const Eigen::VectorXd &y, const Eigen::MatrixXd &H, const Eigen::MatrixXd &P, const Eigen::MatrixXd &R);

}

#endif /* EKF_HELPER_H_ */