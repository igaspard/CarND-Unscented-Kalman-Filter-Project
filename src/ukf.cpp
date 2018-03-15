#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;
  ///* time when the state is true, in us
  time_us_ = 0;
  ///* State dimension
  n_x_ = 5;
  ///* Augmented state dimension
  n_aug_ = 7;
  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;
  ///* Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);
  ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

  R_Lidar = MatrixXd(2, 2);
  R_Lidar << std_laspx_*std_laspx_, 0,
             0, std_laspy_*std_laspy_;

  R_Radar = MatrixXd(3, 3);
  R_Radar << std_radr_*std_radr_, 0, 0,
             0, std_radphi_*std_radphi_, 0,
             0, 0, std_radrd_*std_radrd_;

  NIS_Ladar_ = 0.0;
  NIS_Radar_ = 0.0;

  NIS_FS_Ladar_.open("../output/NIS_Ladar.txt", ios::out);
  NIS_FS_Radar_.open("../output/NIS_Radar.txt", ios::out);

  if ( !NIS_FS_Ladar_.is_open() ) {
    cout << "Fail to open the NIS_Ladar.txt !!!" << endl;
    exit(1);
  }
  if ( !NIS_FS_Radar_.is_open() ) {
    cout << "Fail to open the NIS_Radar.txt !!!" << endl;
    exit(1);
  }
}

UKF::~UKF() {
  NIS_FS_Ladar_.close();
  NIS_FS_Radar_.close();
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_) {
    double init_v = 0.0;
    // init x_ & P_ from the first measurement updates
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      float ro      = meas_package.raw_measurements_[0];
      float theta   = meas_package.raw_measurements_[1];
      float ro_dot  = meas_package.raw_measurements_[2];

      x_ << ro * cos(theta), ro * sin(theta), init_v, 0.0, 0.0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      float px = meas_package.raw_measurements_[0];
      float py = meas_package.raw_measurements_[1];

      x_ << px, py, init_v, 0.0, 0.0;
    }
    P_ << 0.15,    0, 0, 0, 0,
             0, 0.15, 0, 0, 0,
             0,    0, 1, 0, 0,
             0,    0, 0, 1, 0,
             0,    0, 0, 0, 1;

    double weight_0 = lambda_ / (lambda_+n_aug_);
    weights_(0) = weight_0;
    for (int i = 1; i < 2*n_aug_+1; i++) {
      double weight = 0.5 / (lambda_+n_aug_);
      weights_(i) = weight;
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  if (dt > 0.0001) {
    Prediction(dt);
  }

  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    UpdateLidar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    UpdateRadar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  // AugmentedSigmaPoints
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0;
  x_aug(n_x_+1) = 0;

  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_)     = std_a_*std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_*std_yawdd_;
  // Craete square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
  Xsig_aug.fill(0.0);
  Xsig_aug.col(0) = x_aug;
  for (int i = 1; i <= n_aug_; i++) {
    Xsig_aug.col(i)         = x_aug + sqrt(lambda_+n_aug_) * L.col(i-1);
    Xsig_aug.col(i+n_aug_)  = x_aug - sqrt(lambda_+n_aug_) * L.col(i-1);
  }

  // SigmaPointPrediction
  for (int i = 0; i < 2*n_aug_+1; i++) {
    const double px       = Xsig_aug(0, i);
    const double py       = Xsig_aug(1, i);
    const double  v       = Xsig_aug(2, i);
    const double yaw      = Xsig_aug(3, i);
    const double yawd     = Xsig_aug(4, i);
    const double nu_a     = Xsig_aug(5, i);
    const double nu_yawdd = Xsig_aug(6, i);
    // predicted state values
    double px_p, py_p;

    // Avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = px + v/yawd * ( sin(yaw + yawd*delta_t) - sin(yaw) );
      py_p = py + v/yawd * ( cos(yaw) - cos(yaw + yawd*delta_t) );
    } else {
      px_p = px + v * cos(yaw) * delta_t;
      py_p = py + v * sin(yaw) * delta_t;
    }

    double v_p    = v;
    double yaw_p  = yaw + yawd * delta_t;
    double yawd_p = yawd;

    // Add noise
    double dt2 = delta_t * delta_t;
    px_p = px_p + 0.5 * dt2 * cos(yaw) * nu_a;
    py_p = py_p + 0.5 * dt2 * sin(yaw) * nu_a;
     v_p =  v_p + delta_t * nu_a;

    yaw_p   = yaw_p  + 0.5 * dt2 * nu_yawdd;
    yawd_p  = yawd_p + delta_t * nu_yawdd;

    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }
  //cout << "Xsig_pred_ = " << endl << Xsig_pred_ << endl;

  // PredictMeanAndCovariance

  // Predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }
  // Predicted state covariance Matrix
  P_.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // Angle Normalization
    while (x_diff(3) >  M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }

  // cout << "Predicted state" << endl;
  // cout << x_ << endl;
  // cout << "Predicted covariance matrix" << endl;
  // cout << P_ << endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  int n_z = 2;
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
  Zsig.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);

    Zsig(0, i) = px;
    Zsig(1, i) = py;
  }

  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  // Predicted Covariance Matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  S = S + R_Lidar;
  // Calculate cross correlation matrix Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    while (x_diff(3) >  M_PI) x_diff(3) -= 2*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  // Kalman gain K
  MatrixXd K = Tc * S.inverse();
  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];
  VectorXd z_diff = z - z_pred;

  NIS_Ladar_ = z_diff.transpose() * S.inverse() * z_diff;
  NIS_FS_Ladar_ << NIS_Ladar_ << endl;

  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // cout << "UpdateLidar state x_: " << endl << x_ << endl;
  // cout << "UpdateLidar state covariance P_: " << endl << P_ << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  int n_z = 3;
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
  Zsig.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double  v = Xsig_pred_(2, i);
    double yaw= Xsig_pred_(3, i);

    double r = sqrt(px*px + py*py);
    Zsig(0, i) = r;
    Zsig(1, i) = atan2(py, px);
    Zsig(2, i) = (px*cos(yaw)*v + py*sin(yaw)*v) / r;
  }

  MatrixXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  // Predicted Covariance Matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for(int i = 0; i < 2*n_aug_+1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // Angle Normalization
    while (z_diff(1) >  M_PI) z_diff(1) -= 2.0*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  S = S + R_Radar;

  // UpdateState
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // Angle Normalization
    while (z_diff(1) >  M_PI) z_diff(1) -= 2.0*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;

    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    while (x_diff(3) >  M_PI) x_diff(3) -= 2.0*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.0*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  // Kalman Gain
  MatrixXd K = Tc * S.inverse();

  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_[0],
       meas_package.raw_measurements_[1],
       meas_package.raw_measurements_[2];
  // residual
  VectorXd z_diff = z - z_pred;
  while (z_diff(1) >  M_PI) z_diff(1) -= 2.0*M_PI;
  while (z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;

  NIS_Radar_ = z_diff.transpose() * S.inverse() * z_diff;
  NIS_FS_Radar_ << NIS_Radar_ << endl;

  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // cout << "UpdateRadar state x_: " << endl << x_ << endl;
  // cout << "UpdateRadar state covariance P_: " << endl << P_ << endl;
}
