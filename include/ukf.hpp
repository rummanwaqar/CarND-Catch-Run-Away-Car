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

private:
  // set to true in first call of process measurement
  bool is_initialized_;

  // if set to false, laser measurements are ignored
  bool use_laser_;
  // if set to false, radar measurements are ignored
  bool use_radar_;

  // state matrix
  Eigen::VectorXd x_;
  // state covariance
  Eigen::MatrixXd P_;
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

  // predicted sigma points
  Eigen::MatrixXd Xsig_pred_;
  // Weights of sigma points
  Eigen::VectorXd weights_;

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
