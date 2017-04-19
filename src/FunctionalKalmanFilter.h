#ifndef FunctionalKalmanFilter_H_
#define FunctionalKalmanFilter_H_

#include "Eigen/Dense"

class FunctionalKalmanFilter {
public:
	/**
  * Constructor.
  */
  FunctionalKalmanFilter();

  /**
  * Constructor 2.
  */
  FunctionalKalmanFilter(Eigen::VectorXd &x_in_, Eigen::MatrixXd &P_in_, long timestamp);

  /**
  * Destructor.
  */
  virtual ~FunctionalKalmanFilter();

  Eigen::VectorXd getX() const { return x_; }
  Eigen::MatrixXd getP() const { return P_; }
  long getTimestamp() const { return timestamp_; }
  bool isInitialized() const { return initialized_; }

private:
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
  long timestamp_;
  bool initialized_;

};

#endif /* FunctionalKalmanFilter_H_ */