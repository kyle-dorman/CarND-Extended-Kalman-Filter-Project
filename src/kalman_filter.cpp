#include "kalman_filter.h"
#include "tools.h"
#include <math.h>
#include <iostream>

using namespace std;
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

VectorXd h(const VectorXd &x)
{
  float px = x[0];
  float py = x[1];
  float vx = x[2];
  float vy = x[3];

  float c = px * px + py * py;
  float p = pow(c, 0.5);
  float w = atan2(py, px);
  float pv;
  if (c < 0.00001) {
    pv = 0;
  } else {
     pv = (px * vx + py * vy) / pow(c, 0.5);
  }

  VectorXd x_new = VectorXd(3);
  x_new << p, w, pv;
  return x_new;
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
  VectorXd y = z - h(x_);

  while (y[1] > M_PI) {
    y[1] -= 2 * M_PI;
  }
  while (y[1] < -M_PI) {
    y[1] += 2 * M_PI;
  }

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
