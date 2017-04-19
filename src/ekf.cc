#include "ekf_helper.cc"
#include "kalman_filter.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>
#include "measurement_package.h"

using namespace std;
using namespace ekf_helper;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace ekf {

KalmanFilter ProcessMeasurement(MeasurementPackage &measurement_pack, KalmanFilter &kalman_filter)
{
  if (kalman_filter.IsInitialized() == false)
  {
    return Initialize(measurement_pack);
  }

  KalmanFilter prediction = Predict(kalman_filter, 
    F(TimeChange(kalman_filter.Timestamp(), measurement_pack.timestamp_)),
    Q(TimeChange(kalman_filter.Timestamp(), measurement_pack.timestamp_)));

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    return UpdateKalmanFilterWithError(kalman_filter, 
      ExtendedKalmanError(measurement_pack.raw_measurements_, kalman_filter.X()), 
      Hj(measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1],
        measurement_pack.raw_measurements_[2], measurement_pack.raw_measurements_[3]), 
      kalman_filter.P(), 
      R_radar());
  } else
  {
    return UpdateKalmanFilterWithError(kalman_filter, 
      KalmanError(measurement_pack.raw_measurements_, H_laser(), kalman_filter.X()), 
      H_laser(), 
      kalman_filter.P(), 
      R_laser());
  }
}

}
