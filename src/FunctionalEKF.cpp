#include "FunctionalEKF.h"
#include "FunctionalKalmanFilter.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

FunctionalKalmanFilter FunctionalEKF::initialize(const MeasurementPackage &measurement_pack)
{
	MatrixXd P = MatrixXd(4, 4);
		P <<  1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

    VectorXd x = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double ro = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];
      double px = ro * cos(phi);
      double py = ro * sin(phi);

      x << px, py, 0, 0;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      x << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }
    return FunctionalKalmanFilter(x, P, measurement_pack.timestamp_);
}

FunctionalKalmanFilter FunctionalEKF::processMeasurement(MeasurementPackage &measurement_pack, FunctionalKalmanFilter &kalman_filter)
{
	if (kalman_filter.isInitialized() == false)
	{
		return initialize(measurement_pack);
	}

	FunctionalKalmanFilter prediction = predict(kalman_filter, 
		F(dt(kalman_filter.getTimestamp(), measurement_pack.timestamp_)),
		Q(dt(kalman_filter.getTimestamp(), measurement_pack.timestamp_)));

	VectorXd y;

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		y = extendedKalmanError();
	} else
	{
		y = kalmanError();
	}
	return updateWithError(y);
}

MatrixXd FunctionalEKF::R_laser()
{
	MatrixXd R = MatrixXd(2, 2);
	R << 0.0225, 0,
      0, 0.0225;
  return R;
}

MatrixXd FunctionalEKF::R_radar()
{
	MatrixXd R = MatrixXd(3, 3);
	R << 0.09, 0, 0,
			0, 0.0009, 0,
			0, 0, 0.09;
  return R;
}

MatrixXd FunctionalEKF::H_laser()
{
	MatrixXd H = MatrixXd(2, 4);
	H << 1, 0, 0 , 0,
			0, 1, 0, 0;
  return H;
}

double FunctionalEKF::dt(long previous_timestamp, long current_timestamp)
{
	return (current_timestamp - previous_timestamp) / 1000000.0;
}

MatrixXd FunctionalEKF::Q(double dt, double noise_ax = 9, double noise_ay = 9)
{
	MatrixXd Q = MatrixXd(4, 4);
  double t2 = dt * dt;
  double t3 = t2 * dt / 2;
  double t4 = t3 * dt / 2;

  Q << t4 * noise_ax, 0, t3 * noise_ax, 0,
  		0, t4 * noise_ay, 0, t3 * noise_ay,
  		t3 * noise_ax, 0, t2 * noise_ax, 0,
  		0, t3 * noise_ay, 0, t2 * noise_ay;

  return Q;
}

MatrixXd FunctionalEKF::F(double dt)
{
  MatrixXd F = MatrixXd(4, 4);
  F <<  1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0 , 0, 1;

  return F;
}

MatrixXd FunctionalEKF::I(int size)
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

MatrixXd FunctionalEKF::Hj(double px, double py, double vx, double vy)
{
  MatrixXd Hj = MatrixXd(3, 4);
  double c = px * px + py * py;

  if (fabs(c) < 0.00001)
  {
    Hj << 0,0,0,0,
          0,0,0,0,
          0,0,0,0;
  } else 
  {
    double d = (vx * py - vy * px) * py;
    double e = (vy * px - vx * py) * px;

    Hj << px / pow(c, 0.5), py / pow(c, 0.5), 0, 0,
          -py / c, px / c, 0, 0,
          d / pow(c, 1.5), e / pow(c, 1.5), px / pow(c, 0.5), py / pow(c, 0.5);
  }

  return Hj;
}

VectorXd FunctionalEKF::h(const VectorXd &x)
{
  double px = x[0];
  double py = x[1];
  double vx = x[2];
  double vy = x[3];

  double c = px * px + py * py;
  double p = pow(c, 0.5);
  double w = atan2(py, px);
  double pv;
  if (c < 0.00001) {
    pv = 0;
  } else {
     pv = (px * vx + py * vy) / pow(c, 0.5);
  }

  VectorXd x_new = VectorXd(3);
  x_new << p, w, pv;
  return x_new;
}

FunctionalKalmanFilter FunctionalEKF::predict(FunctionalKalmanFilter &kalman_filter, const MatrixXd &F, const MatrixXd &Q, long timestamp)
{
	return FunctionalKalmanFilter(
		F * kalman_filter.getX(),
		F * kalman_filter.getP() * F.transpose() + Q,
		timestamp
	);
}

VectorXd FunctionalEKF::kalmanError(const VectorXd &z, const MatrixXd &H, const VectorXd &x)
{
	return z - H * x;
}

VectorXd FunctionalEKF::extendedKalmanError(const VectorXd &z, const VectorXd &x)
{
	VectorXd y = z - h(x);

  while (y[1] > M_PI) {
    y[1] -= 2 * M_PI;
  }
  while (y[1] < -M_PI) {
    y[1] += 2 * M_PI;
  }
  return y;
}

FunctionalKalmanFilter FunctionalEKF::updateWithError(const VectorXd &y, const MatrixXd &H, const MatrixXd &P, const MatrixXd &R, long timestamp)
{
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P * Ht + R;
  MatrixXd K = P * Ht * S.inverse();

  return FunctionalKalmanFilter(
  	x + K * y,
  	(identity(4) - K * H) * P,
  	timestamp
  );
}


