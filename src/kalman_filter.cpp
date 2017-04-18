#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd identity(int size)
{
  MatrixXd I = MatrixXd(size, size);
  for (int i = 0; i < size; i++)
  {
    for (int j = 0; j < size; j++)
    {
      if (i == j) {
        I(i, j) = 1;
      } else {
        I(i, j) = 0;
      }
    }
  }
  return I;
}

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in; // poistion & velocity
  P_ = P_in; //
  F_ = F_in; // State transistion function
  H_ = H_in;
  R_ = R_in; // measurement noise covariance matrix
  Q_ = Q_in; // process noise covariance matrix
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  MatrixXd Ht = H_.transpose();
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  x_ = x_ + K * y;

  P_ = (identity(4) - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  MatrixXd Ht = H_.transpose();
  // (3, 1) - (3, 4) * (4, 1);
  VectorXd y = z - H_ * x_;
  // S = H * P * HT + R
  // (3, 3) = (3, 4) * (4, 4) * (4 * 3) + (3, 3);
  MatrixXd S = H_ * P_ * Ht + R_;
  // K = P * HT * S-1
  // (4, 3) = (4, 4) * (4, 3) * (3, 3)
  MatrixXd K = P_ * Ht * S.inverse();

  // x = x + Ky
  // (4, 1) = (4, 1) + (4, 3) * (3, 1);
  x_ = x_ + K * y;
  // P = (I − KH)P
  // (4, 4) = ((4, 4) - (4, 3) * (3, 4)) * (4, 4);
  P_ = (identity(4) - K * H_) * P_;
}
