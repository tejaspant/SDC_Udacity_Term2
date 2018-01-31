#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  //measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    VectorXd xin(4);
    MatrixXd Pin(4,4);
    MatrixXd Fin(4,4);
    MatrixXd Hin(2,4);
    MatrixXd Hjin(3,4);
    MatrixXd Rin(2,2);
    MatrixXd Rekf_in(3,3);

    ekf_.P_ = MatrixXd(4,4); //Process covariance matrix

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho =  measurement_pack.raw_measurements_[0];
      float phi =  measurement_pack.raw_measurements_[1];
      float rhodot =  measurement_pack.raw_measurements_[2];

      float xR = rho * cos(phi);
      float yR = rho * sin(phi);
      float vxR = rhodot * cos(phi);
      float vyR = rhodot * sin(phi);

      xin  << xR, yR, vxR, vyR;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      xin  << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    Pin << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1000, 0,
           0, 0, 0, 1000;

    Fin << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;

    Hjin = tools.CalculateJacobian(xin);

    ekf_.Init(xin, Pin, Fin, H_laser_, Hjin, R_laser_, R_radar_);

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt3 * dt;
  float noise_ax = 9;
  float noise_ay = 9;

  // Calculate state transition matrix F
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  // Calculate process noise covariance matrix Q
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << dt4/4*noise_ax, 0, dt3/2*noise_ax, 0,
             0, dt4/4*noise_ay, 0, dt3/2*noise_ay,
             dt3/2*noise_ax, 0, dt2*noise_ax, 0,
             0, dt3/2*noise_ay, 0, dt2*noise_ay;


  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Update Jacobian
    ekf_.Hj_ = tools.CalculateJacobian(ekf_.x_);
    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }


  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
