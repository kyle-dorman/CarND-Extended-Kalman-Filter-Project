#ifndef FunctionalEKF_H_
#define FunctionalEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include "FunctionalKalmanFilter.h"

class FunctionalEKF {
public:
  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  FunctionalKalmanFilter static processMeasurement(MeasurementPackage &measurement_pack, FunctionalKalmanFilter &kalman_filter);

private:
  static Eigen::MatrixXd R_laser(); 
  static Eigen::MatrixXd R_radar();
  static Eigen::MatrixXd H_laser();
 
 	static double dt(long previous_timestamp, long current_timestamp);
 	static Eigen::MatrixXd Q(double dt, float noise_ax = 9, float noise_ay = 9);
 	static Eigen::MatrixXd F(double dt);
 	static Eigen::MatrixXd I(int size);
 	static Eigen::MatrixXd Hj(float px, float py, float vx, float vy);
 	static Eigen::VectorXd h(const Eigen::VectorXd &x);

 	static FunctionalKalmanFilter initialize(const MeasurementPackage &measurement_pack);
 	static FunctionalKalmanFilter predict(FunctionalKalmanFilter &kalman_filter, const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q, long timestamp);
 	static Eigen::VectorXd kalmanError(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::VectorXd &x);
 	static Eigen::VectorXd extendedKalmanError(const Eigen::VectorXd &z, const Eigen::VectorXd &x);
 	static FunctionalKalmanFilter updateWithError(const Eigen::VectorXd &y, const Eigen::MatrixXd &H, const Eigen::MatrixXd &P, const Eigen::MatrixXd &R, long timestamp);

};

#endif /* FunctionalEKF_H_ */
