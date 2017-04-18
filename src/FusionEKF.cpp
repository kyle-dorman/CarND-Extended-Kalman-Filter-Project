#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

MatrixXd update_Q(long previous_timestamp, long current_timestamp, float noise_ax = 9, float noise_ay = 9)
{
  MatrixXd Q = MatrixXd(4, 4);
  double dt = (current_timestamp - previous_timestamp) / 1000000.0;
  float t2 = dt * dt;
  float t3 = t2 * dt / 2;
  float t4 = t3 * dt / 2;

  Q << t4 * noise_ax, 0, t3 * noise_ax, 0,
  0, t4 * noise_ay, 0, t3 * noise_ay,
  t3 * noise_ax, 0, t2 * noise_ax, 0,
  0, t3 * noise_ay, 0, t2 * noise_ay;

  return Q;
}

MatrixXd update_F(long previous_timestamp, long current_timestamp)
{
  MatrixXd F = MatrixXd(4, 4);
  double dt = (current_timestamp - previous_timestamp) / 1000000.0;
  F <<  1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0 , 0, 1;

  return F;
}

MatrixXd update_Hj(float px, float py, float vx, float vy)
{
  MatrixXd Hj = MatrixXd(3, 4);
  float c = px * px + py * py;

  if (fabs(c) < 0.00001)
  {
    Hj << 0,0,0,0,
          0,0,0,0,
          0,0,0,0;
  } else
  {
    float d = (vx * py - vy * px) * py;
    float e = (vy * px - vx * py) * px;

    Hj << px / pow(c, 0.5), py / pow(c, 0.5), 0, 0,
          -py / c, px / c, 0, 0,
          d / pow(c, 1.5), e / pow(c, 1.5), px / pow(c, 0.5), py / pow(c, 0.5);
  }

  return Hj;
}

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1, 0, 0 , 0,
              0, 1, 0, 0;

  Hj_ <<  0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;

  VectorXd x = VectorXd(4);
  x << 0, 0, 0, 0;

  MatrixXd P = MatrixXd(4, 4);
  P <<  1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  MatrixXd F = update_F(0, 0);
  MatrixXd Q = update_Q(0, 0);

  ekf_.Init(x, P, F, H_laser_, R_laser_, Q);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;

    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float x = ro * cos(phi);
      float y = ro * sin(phi);

      ekf_.x_ << x, y, 0, 0;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  ekf_.Q_ = update_Q(previous_timestamp_, measurement_pack.timestamp_);
  ekf_.F_ = update_F(previous_timestamp_, measurement_pack.timestamp_);
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    cout << "RADER" << endl;
    Hj_ = update_Hj(ekf_.x_[0], ekf_.x_[1], ekf_.x_[2], ekf_.x_[3]);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    cout << "LASER" << endl;
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}


















