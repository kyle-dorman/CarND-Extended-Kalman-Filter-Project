#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {
public:
  /**
  * Build empty KalmanFilter
  */
  KalmanFilter static Empty();

  /**
  * Constructor.
  */
  KalmanFilter(const Eigen::VectorXd &x_in_, const Eigen::MatrixXd &P_in_, long timestamp);

  /**
  * Destructor.
  */
  virtual ~KalmanFilter();

  Eigen::VectorXd X() const { return x_; }
  Eigen::MatrixXd P() const { return P_; }
  long Timestamp() const { return timestamp_; }
  bool IsInitialized() const { return initialized_; }

private:
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
  long timestamp_;
  bool initialized_;

};

#endif /* KALMAN_FILTER_H_ */