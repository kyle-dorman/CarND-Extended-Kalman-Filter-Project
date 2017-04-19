#include "ekf_helper.h"
#include "kalman_filter.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>
#include "measurement_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace ekf {

inline KalmanFilter ProcessMeasurement(MeasurementPackage &measurement_pack, KalmanFilter &kalman_filter)
{
  if (kalman_filter.IsInitialized() == false)
  {
    return ekf_helper::Initialize(measurement_pack);
  }

  KalmanFilter prediction = ekf_helper::Predict(kalman_filter, 
    ekf_helper::F(ekf_helper::TimeChange(kalman_filter.Timestamp(), measurement_pack.timestamp_)),
    ekf_helper::Q(ekf_helper::TimeChange(kalman_filter.Timestamp(), measurement_pack.timestamp_), 9, 9));

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    VectorXd x = kalman_filter.X();

    return ekf_helper::UpdateKalmanFilterWithError(kalman_filter, 
      ekf_helper::ExtendedKalmanError(measurement_pack.raw_measurements_, kalman_filter.X()), 
      ekf_helper::Hj(x[0], x[1], x[2], x[3]), 
      kalman_filter.P(), 
      ekf_helper::R_radar());
  } else
  {
    return ekf_helper::UpdateKalmanFilterWithError(kalman_filter, 
      ekf_helper::KalmanError(measurement_pack.raw_measurements_, ekf_helper::H_laser(), kalman_filter.X()), 
      ekf_helper::H_laser(), 
      kalman_filter.P(), 
      ekf_helper::R_laser());
  }
}

}
