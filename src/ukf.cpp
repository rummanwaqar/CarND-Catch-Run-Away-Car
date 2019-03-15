#include "ukf.hpp"
#include <iostream>
Ukf::Ukf() : is_initialized_(false) {
  use_laser_ = true;
  use_radar_ = true;

  // state = [x y vel yaw yaw_rate]
  n_x_ = 5;
  n_aug_ = 7;
  x_ = Eigen::VectorXd(n_x_);
  P_ = Eigen::MatrixXd(n_x_, n_x_);

  // set process noise
  // std_a_ = 3.0;
  // std_yawdd_ = M_PI/2.;

  std_a_ = 0.2;
  std_yawdd_ = 0.2;
  // set lidar measurement noise
  std_laspx_ = 0.15;
  std_laspy_ = 0.15;
  // set radar measurement noise
  std_radr_ = 0.3;
  std_radphi_ = 0.03;
  std_radrd_ = 0.3;

  // predicted sigma points
  Xsig_pred_ = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);

  // initialize sigma weights
  double lambda = 3 - n_aug_;
  weights_ = Eigen::VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda / (lambda + n_aug_);
  for(int i=0; i < 2 * n_aug_; i++) {
    weights_(i+1) = 0.5 / (lambda + n_aug_);
  }

  x_ <<   5.7441,
         1.3800,
         2.2049,
         0.5015,
         0.3528;

  // create example covariance matrix
  P_ <<    0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;
}

void Ukf::prediction(double delta_t) {
  if(is_initialized_) {
    // create augmented state
    Eigen::VectorXd x_aug = Eigen::VectorXd::Zero(n_aug_);
    x_aug.head(n_x_) = x_;
    Eigen::MatrixXd P_aug = Eigen::MatrixXd::Zero(n_aug_, n_aug_);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(5, 5) = std_a_ * std_a_;
    P_aug(6, 6) = std_yawdd_ * std_yawdd_;

    Eigen::MatrixXd Xsig_aug = calcSigmaPoints(x_aug, P_aug, 3 - n_aug_);

    // predict sigma points (process model)
    for(int i=0; i<2 * n_aug_ + 1; i++) {
      double x = Xsig_aug(0, i);
      double y = Xsig_aug(1, i);
      double v = Xsig_aug(2, i);
      double yaw = Xsig_aug(3, i);
      double yaw_rate = Xsig_aug(4, i);
      double a = Xsig_aug(5, i);
      double yaw_dd = Xsig_aug(6, i);

      // px and py
      if(yaw_rate == 0) {
          Xsig_pred_(0, i) = x + v * cos(yaw) * delta_t;
          Xsig_pred_(1, i) = y + v * sin(yaw) * delta_t;
      } else {
          Xsig_pred_(0, i) = x + v / yaw_rate * (sin(yaw + yaw_rate * delta_t) - sin(yaw));
          Xsig_pred_(1, i) = y + v / yaw_rate * (-cos(yaw + yaw_rate * delta_t) + cos(yaw));
      }
      // velocity
      Xsig_pred_(2, i) = v;
      // yaw
      Xsig_pred_(3, i) = yaw + yaw_rate * delta_t;
      // yaw_rate
      Xsig_pred_(4, i) = yaw_rate;

      // add noise
      Xsig_pred_(0, i) += delta_t * delta_t / 2. * cos(yaw) * a;
      Xsig_pred_(1, i) += delta_t * delta_t / 2. * sin(yaw) * a;
      Xsig_pred_(2, i) += delta_t * Xsig_aug(5,i);
      Xsig_pred_(3, i) += delta_t * delta_t / 2. * yaw_dd;
      Xsig_pred_(4, i) += delta_t * yaw_dd;

    }
    // Eigen::VectorXd x = calcState(Xsig_pred_);
    //
    // Eigen::MatrixXd P = calcCovariance(Xsig_pred_, [&](Eigen::VectorXd Xsig) {
    //   Eigen::VectorXd diff = Xsig - x;
    //   // angle normalization
    //   while (diff(3)> M_PI) diff(3)-=2.*M_PI;
    //   while (diff(3)<-M_PI) diff(3)+=2.*M_PI;
    //   return diff;
    // });
    // std::cout << P << std::endl;
  }
}

Eigen::MatrixXd Ukf::calcSigmaPoints(Eigen::VectorXd x, Eigen::MatrixXd P, double lambda) {
  int n = x.size();
  // create sigma point matrix
  Eigen::MatrixXd Xsig = Eigen::MatrixXd(n, 2 * n + 1);
  Eigen::MatrixXd A = P.llt().matrixL(); // calculate sqrt P
  Xsig.col(0) = x;
  for(int i=0; i < n; i++) {
    Xsig.col(i+1) = x + sqrt(lambda + n) * A.col(i);
    Xsig.col(i+1+n) = x - sqrt(lambda + n) * A.col(i);
  }
  return Xsig;
}

Eigen::VectorXd Ukf::calcState(Eigen::MatrixXd Xsig) {
  double n_states = Xsig.rows();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n_states);
  for(int i=0; i<2 * n_aug_ + 1; i++) {
      x = x + weights_(i) * Xsig.col(i);
  }
  return x;
}

Eigen::MatrixXd Ukf::calcCovariance(Eigen::MatrixXd Xsig,
  std::function<Eigen::VectorXd(Eigen::VectorXd Xsig)> diff_fn) {
  double n_states = Xsig.rows();
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n_states, n_states);
  for(int i=0; i < 2 * n_aug_ + 1; i++) {
    // calculate diff between sigma col and x
    Eigen::VectorXd diff = diff_fn(Xsig.col(i));
    P = P + weights_(i) * diff * diff.transpose();
  }
  return P;
}
