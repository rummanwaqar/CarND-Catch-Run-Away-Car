#pragma once

#include <vector>
#include <string>
#include <cmath>
#include "Eigen/Dense"
#include "types.hpp"

class Ukf {
public:
  Ukf();

  virtual ~Ukf() = default;

  /**
   * predicts sigma points, the state, and the state covariance
   * @param delta_t Time between k and k+1 in s
   */
  void prediction(double delta_t);

  /**
   * @param {MeasurementPackage} meas_package The latest measurement data of
   * either radar or laser.
   */
  void processMeasurement(MeasurementPackage measurement);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void updateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void updateRadar(MeasurementPackage meas_package);

  // state matrix
  Eigen::VectorXd x_;
  // state covariance
  Eigen::MatrixXd P_;
  // NIS values for debug
  double NIS_radar_;
  double NIS_lidar_;

private:
  // set to true in first call of process measurement
  bool is_initialized_;

  // if set to false, laser measurements are ignored
  bool use_laser_;
  // if set to false, radar measurements are ignored
  bool use_radar_;

  // State dimension
  int n_x_;
  // Augmented state dimension
  int n_aug_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;
  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;
  // Laser measurement noise standard deviation x in m
  double std_laspx_;
  // Laser measurement noise standard deviation y in m
  double std_laspy_;
  // Radar measurement noise standard deviation radius in m
  double std_radr_;
  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;
  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;

  // predicted sigma points
  Eigen::MatrixXd Xsig_pred_;
  // Weights of sigma points
  Eigen::VectorXd weights_;

  // previous timestamp (used for delta_t calc)
  long long previous_timestamp;

  /*
   * calculates sigma points
   */
  Eigen::MatrixXd calcSigmaPoints(Eigen::VectorXd x, Eigen::MatrixXd P, double lambda);

  /*
   * calculate state from sigma points
   */
  Eigen::VectorXd calcState(Eigen::MatrixXd Xsig);

  /*
   * calculate covariance from sigma points
   * needs a function that calcs difference between sigma column and x
   */
  Eigen::MatrixXd calcCovariance(Eigen::MatrixXd Xsig,
    std::function<Eigen::VectorXd(Eigen::VectorXd Xsig)> diff_fn);
};
