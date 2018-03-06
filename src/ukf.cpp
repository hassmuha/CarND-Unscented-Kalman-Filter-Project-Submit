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
  // if not initialized
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  //TODO : Change these process noise value according to lecture
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.0625; //using v = rw and putting value of v = 1 (tangential velocity (coming from std_a_ as this we expect the cycle can maximum accelerate)) r = 15 (coming from lecture)

  //NOTE : DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //set state dimension
  n_x_ = 5;

  //set augmented dimension
  n_aug_ = n_x_ + 2;

  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  lambda_ = 3 - n_aug_;

  //initialize weights as it will remain unchnaged throughout the application
  weights_ = VectorXd(2*n_aug_+1);
  //set weights
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
      if (i ==0) {
          weights_(i) = lambda_/(lambda_ + n_aug_);
      } else {
          weights_(i) = 1/(2*(lambda_ + n_aug_));
      }
  }
}

UKF::~UKF() {}

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

    // first measurement
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro;
      float theta;
      float ro_dot;
      ro = meas_package.raw_measurements_[0];
      theta = meas_package.raw_measurements_[1];
      ro_dot = meas_package.raw_measurements_[2];


      float px;
      float py;

      // initializing px and py from ro and theta
      px = ro * cos(theta);
      py = ro * sin(theta);

      // TODO updating the x_ and P_
      //ekf_.x_ << px, py, 5, 0;
      x_ << px, py, 0, 0, 0;
      P_.setZero(n_x_, n_x_);
      P_(0,0) = (std_radr_ + std_radphi_) * (std_radr_ + std_radphi_); //
      P_(1,1) = (std_radr_ + std_radphi_) * (std_radr_ + std_radphi_);
      P_(2,2) = 1; //we cannot estimate this so keep it like this
      P_(3,3) = 1; //we cannot estimate this so keep it like this
      P_(4,4) = 1; //we cannot estimate this so keep it like this

      time_us_ = meas_package.timestamp_;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //set the state with the initial location and zero velocity
      float px;
      float py;

      // TODO updating the x_ and P_
      px = meas_package.raw_measurements_[0];
      py = meas_package.raw_measurements_[1];
		  x_ << px, py, 0, 0, 0;
      P_.setZero(n_x_, n_x_);
      P_(0,0) = std_laspx_ * std_laspx_;
      P_(1,1) = std_laspy_ * std_laspy_;
      P_(2,2) = 1; //we cannot estimate this so keep it like this
      P_(3,3) = 1; //we cannot estimate this so keep it like this
      P_(4,4) = 1; //we cannot estimate this so keep it like this
      time_us_ = meas_package.timestamp_;

    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    cout << "x_ = " << x_ << endl;
    cout << "P_ = " << P_ << endl;
    return;
  }
  //compute the time elapsed between the current and previous measurements
  float delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
  time_us_ = meas_package.timestamp_;

  GenerateSigmaPoints();
  SigmaPointPrediction(delta_t);
  PredictMeanAndCovariance();

  // Measurement Update
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // For Radar
	  UpdateRadar(meas_package);
  } else {
    // For Laser
	  UpdateLidar(meas_package);
  }
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;

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

  GenerateSigmaPoints();
  SigmaPointPrediction(delta_t);
  PredictMeanAndCovariance();
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

  //start of PredictLidarMeasurement
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 2;
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //measurement noise R
  MatrixXd R = MatrixXd(n_z,n_z);
  R(0,0) = std_laspx_ * std_laspx_;
  R(1,1) = std_laspy_ * std_laspy_;

  //transform sigma points into measurement space
  VectorXd px_array = Xsig_pred_.row(0);
  VectorXd py_array = Xsig_pred_.row(1);
  VectorXd v_array = Xsig_pred_.row(2);
  VectorXd si_array = Xsig_pred_.row(3);
  VectorXd sid_array = Xsig_pred_.row(4);
  //intitialize result array
  Zsig.row(0) = px_array;
  Zsig.row(1) = py_array;
  //calculate mean predicted measurement
  z_pred= Zsig * weights_;
  //calculate innovation covariance matrix S
  MatrixXd mean_sub = Zsig.colwise() - z_pred;
  S = ((mean_sub * weights_.asDiagonal()) * mean_sub.transpose()) + R;
  //end of PredictRadarMeasurement


  // Start of Update
  VectorXd z = meas_package.raw_measurements_;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  //calculate cross correlation matrix
  MatrixXd mean_sub_Zsig = Zsig.colwise() - z_pred;
  MatrixXd mean_sub_Xsig_pred = Xsig_pred_.colwise() - x_;
  Tc = (mean_sub_Xsig_pred * weights_.asDiagonal()) * mean_sub_Zsig.transpose();
  //calculate Kalman gain K;
  MatrixXd Si = S.inverse();
  MatrixXd K = Tc * Si;

  //update state mean and covariance matrix
  VectorXd y = z - z_pred;
  x_ = x_ + (K * y);

  P_ = P_ - (K*S*K.transpose());
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
  //start of PredictRadarMeasurement
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //measurement noise R
  MatrixXd R = MatrixXd(n_z,n_z);
  R(0,0) = std_radr_ * std_radr_;
  R(1,1) = std_radphi_ * std_radphi_;
  R(2,2) = std_radrd_ * std_radrd_;

  //transform sigma points into measurement space
  VectorXd px_array = Xsig_pred_.row(0);
  VectorXd py_array = Xsig_pred_.row(1);
  VectorXd v_array = Xsig_pred_.row(2);
  VectorXd si_array = Xsig_pred_.row(3);
  VectorXd sid_array = Xsig_pred_.row(4);
  //intitialize result array
  VectorXd radr_array = VectorXd(2*n_aug_+1);
  VectorXd radphi_array = VectorXd(2*n_aug_+1);
  VectorXd radrd_array = VectorXd(2*n_aug_+1);

  radr_array = sqrt((px_array.array()*px_array.array()) + (py_array.array()*py_array.array()));
  for( int i = 0; i < 2*n_aug_+1; i++ ) {
      radphi_array(i) = atan2(py_array(i),px_array(i));
   }
  //FIXME : divide by zero in equation below
  radrd_array = (px_array.array()*cos(si_array.array())*v_array.array() + py_array.array()*sin(si_array.array())*v_array.array())/radr_array.array();

  Zsig.row(0) = radr_array;
  Zsig.row(1) = radphi_array;
  Zsig.row(2) = radrd_array;
  //calculate mean predicted measurement
  z_pred= Zsig * weights_;
  //calculate innovation covariance matrix S
  MatrixXd mean_sub = Zsig.colwise() - z_pred;
  S = ((mean_sub * weights_.asDiagonal()) * mean_sub.transpose()) + R;
  //end of PredictRadarMeasurement


  // Start of Update
  VectorXd z = meas_package.raw_measurements_;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  //calculate cross correlation matrix
  MatrixXd mean_sub_Zsig = Zsig.colwise() - z_pred;
  MatrixXd mean_sub_Xsig_pred = Xsig_pred_.colwise() - x_;
  Tc = (mean_sub_Xsig_pred * weights_.asDiagonal()) * mean_sub_Zsig.transpose();
  //calculate Kalman gain K;
  MatrixXd Si = S.inverse();
  MatrixXd K = Tc * Si;

  //update state mean and covariance matrix
  VectorXd y = z - z_pred;
  x_ = x_ + (K * y);

  P_ = P_ - (K*S*K.transpose());
}


void UKF::GenerateSigmaPoints() {

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug << x_,
        0,
        0;
  //create augmented covariance matrix
  P_aug.setZero(n_aug_, n_aug_);
  //Upper part of the P_aug is P
  P_aug.block(0,0,n_x_,n_x_) = P_;
  // include process noise in the P_aug matrix
  P_aug(n_x_,n_x_) = std_a_*std_radr_; //std_a_
  P_aug(n_x_+1,n_x_+1) = std_yawdd_*std_yawdd_; //std_yawdd_

  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();
  //create augmented sigma points
  Xsig_aug_.col(0) = x_aug;
  Xsig_aug_.block(0,1,n_aug_,n_aug_) = (A*sqrt(lambda_+n_aug_)).colwise() + x_aug;
  Xsig_aug_.block(0,n_aug_+1,n_aug_,n_aug_) = (-A*sqrt(lambda_+n_aug_)).colwise() + x_aug;
}


void UKF::SigmaPointPrediction(double delta_t) {

  double p_x,p_y,v,yaw,yawd,nu_a,nu_yawdd;

  double delta_t2_half = delta_t*delta_t/2;
  double sin_yaw,cos_yaw,sin_yaw_yawrate,cos_yaw_yawrate,v_by_yawrate;

  //create predicted sigma point vector
  VectorXd x_pred = VectorXd(5);
  for( int idx = 0; idx < 2 * n_aug_ + 1; idx = idx + 1 ) {

      p_x = Xsig_aug_(0,idx);
      p_y = Xsig_aug_(1,idx);
      v = Xsig_aug_(2,idx);
      yaw = Xsig_aug_(3,idx);
      yawd = Xsig_aug_(4,idx);
      nu_a = Xsig_aug_(5,idx);
      nu_yawdd = Xsig_aug_(6,idx);

      //predict sigma points
      sin_yaw = sin(yaw);
      cos_yaw = cos(yaw);
      sin_yaw_yawrate = sin(yaw + yawd*delta_t);
      cos_yaw_yawrate = cos(yaw + yawd*delta_t);
      v_by_yawrate = v/yawd;
      //avoid division by zero
      if (yawd == 0){
          x_pred << p_x + v*cos_yaw*delta_t + delta_t2_half*cos_yaw*nu_a,
            p_y + v*sin_yaw*delta_t + delta_t2_half*sin_yaw*nu_a,
            v + 0 + delta_t*nu_a,
            yaw + 0 + delta_t2_half*nu_yawdd,
            yawd + 0 + delta_t*nu_yawdd;
      } else {
          x_pred << p_x + v_by_yawrate*(sin_yaw_yawrate - sin_yaw) + delta_t2_half*cos_yaw*nu_a,
            p_y + v_by_yawrate*(-cos_yaw_yawrate + cos_yaw) + delta_t2_half*sin_yaw*nu_a,
            v + 0 + delta_t*nu_a,
            yaw + yawd*delta_t + delta_t2_half*nu_yawdd,
            yawd + 0 + delta_t*nu_yawdd;
      }
      //write predicted sigma points into right column
      Xsig_pred_.col(idx) = x_pred;
  }

}

void UKF::PredictMeanAndCovariance() {
  //predict state mean
  x_ = Xsig_pred_ * weights_;
  //predict state covariance matrix
  MatrixXd mean_sub = Xsig_pred_.colwise() - x_;

  P_ = (mean_sub * weights_.asDiagonal()) * mean_sub.transpose();
}
