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
  std_a_ = 3.0;
  std_yawdd_ = M_PI;
  // set lidar measurement noise
  std_laspx_ = 0.15;
  std_laspy_ = 0.15;
  R_laser_ = Eigen::MatrixXd(2,2);
  R_laser_ << std_laspx_*std_laspx_, 0, 0, std_laspy_*std_laspy_;
  // set radar measurement noise
  std_radr_ = 0.3;
  std_radphi_ = 0.03;
  std_radrd_ = 0.3;
  R_radar_ = Eigen::MatrixXd(3,3);
  R_radar_ << std_radr_*std_radr_, 0,0,
              0, std_radphi_*std_radphi_, 0,
              0,0, std_radrd_*std_radrd_;

  // predicted sigma points
  Xsig_pred_ = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);

  // initialize sigma weights
  double lambda = 3 - n_aug_;
  weights_ = Eigen::VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda / (lambda + n_aug_);
  for(int i=0; i < 2 * n_aug_; i++) {
    weights_(i+1) = 0.5 / (lambda + n_aug_);
  }

  NIS_lidar_ = NIS_radar_ = 0.0;
}

void Ukf::prediction(double delta_t) {
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
    if(fabs(yaw_rate) > 0.001) {
      Xsig_pred_(0, i) = x + v / yaw_rate * (sin(yaw + yaw_rate * delta_t) - sin(yaw));
      Xsig_pred_(1, i) = y + v / yaw_rate * (-cos(yaw + yaw_rate * delta_t) + cos(yaw));
    } else {
      Xsig_pred_(0, i) = x + v * cos(yaw) * delta_t;
      Xsig_pred_(1, i) = y + v * sin(yaw) * delta_t;
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

  x_ = calcState(Xsig_pred_);
  P_ = calcCovariance(Xsig_pred_, [&](Eigen::VectorXd Xsig_col) {
    Eigen::VectorXd x_diff = Xsig_col - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    return x_diff;
  });
}

void Ukf::processMeasurement(MeasurementPackage measurement) {
  if(!is_initialized_) {
    if(measurement.sensor_type == MeasurementPackage::LASER) {
      x_ << measurement.raw_measurements(0),
            measurement.raw_measurements(0),
            0,0,0;
      Eigen::VectorXd initial_var(5);
      initial_var << .15, .15, 1., 1., 1.;
      P_ = initial_var.asDiagonal();
      std::cout << "Initialized with LIDAR message:\n" << x_ << std::endl;
    } else if(measurement.sensor_type == MeasurementPackage::RADAR) {
      // set initial state using radar values (convert ρ,φ -> px, py, v)
      float rho = measurement.raw_measurements(0);
      float phi = measurement.raw_measurements(1);
      double rho_dot = measurement.raw_measurements(2);
      double px = rho * cos(phi);
      double py = rho * sin(phi);
      double vx = rho_dot * cos(phi);
      double vy = rho_dot * sin(phi);
      double v = sqrt(vx * vx + vy * vy);
      x_ << px, py, v, 0, 0;
      Eigen::VectorXd initial_var(5);
      initial_var << 0.15, 0.15, 0.15, 1., 1.;
      P_ = initial_var.asDiagonal();
      std::cout << "Initialized with RADAR message:\n" << x_ << std::endl;
    }
    previous_timestamp = measurement.timestamp;
    is_initialized_ = true;
    return;
  }

  double dt = (measurement.timestamp - previous_timestamp) / 1000000.0;
  // prediction
  prediction(dt);

  // update
  if(measurement.sensor_type == MeasurementPackage::LASER && use_laser_) {
    updateLidar(measurement);
  } else if(measurement.sensor_type == MeasurementPackage::RADAR && use_radar_) {
    updateRadar(measurement);
  }
  //
  previous_timestamp = measurement.timestamp;
}

void Ukf::updateLidar(MeasurementPackage meas_package) {
  int n_z = 2;
  // create matrix for sigma points in measurement space
  Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z, 2 * n_aug_ + 1);
  // predict sigma points
  for(int i=0; i<2 * n_aug_ + 1; i++) {
    Zsig(0, i) = Xsig_pred_(0, i); // px
    Zsig(1, i) = Xsig_pred_(1, i); // py
  }

  Eigen::VectorXd z_pred = calcState(Zsig);
  Eigen::MatrixXd S = calcCovariance(Zsig, [&](Eigen::VectorXd Zsig_col) {
    return Zsig_col - z_pred;
  });
  S = S + R_laser_;

  // update
  // create matrix for cross correlation Tc
  Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(n_x_, n_z);
  for(int i=0; i < 2 * n_aug_ + 1; i++) {
    // residual
    Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // calculate kalman gain
  Eigen::MatrixXd K = Tc * S.inverse();

  // update state mean and covariance matrix
  Eigen::VectorXd z_diff = meas_package.raw_measurements - z_pred;
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // update NIS
  NIS_lidar_ = z_diff.transpose() * S.inverse() * z_diff;
}

void Ukf::updateRadar(MeasurementPackage meas_package) {
  int n_z = 3;
  // create matrix for sigma points in measurement space
  Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z, 2 * n_aug_ + 1);
  // predict sigma points
  for(int i=0; i<2 * n_aug_ + 1; i++) {
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    // calculate vx and vy
    double vx = cos(yaw) * v;
    double vy = sin(yaw) * v;

    // store values that will be reused for efficiency
    double divisor = sqrt(px*px + py*py);

    // measurement model for radar
    Zsig(0, i) = divisor;
    Zsig(1, i) = atan2(py, px);
    Zsig(2, i) = (px*vx + py*vy) / divisor;
  }

  Eigen::VectorXd z_pred = calcState(Zsig);
  Eigen::MatrixXd S = calcCovariance(Zsig, [&](Eigen::VectorXd Zsig_col) {
    Eigen::VectorXd diff = Zsig_col - z_pred;
    // angle normalization
    while (diff(1)> M_PI) diff(1)-=2.*M_PI;
    while (diff(1)<-M_PI) diff(1)+=2.*M_PI;
    return diff;
  });
  S = S + R_radar_;

  // update
  // create matrix for cross correlation Tc
  Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(n_x_, n_z);
  for(int i=0; i < 2 * n_aug_ + 1; i++) {
    // residual
    Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // calculate kalman gain
  Eigen::MatrixXd K = Tc * S.inverse();

  // update state mean and covariance matrix
  Eigen::VectorXd z_diff = meas_package.raw_measurements - z_pred;
  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // update NIS
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
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
