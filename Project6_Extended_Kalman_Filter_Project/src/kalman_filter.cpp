#include "kalman_filter.h"
#include <math.h>
#include <iostream>

#define PI 3.14159265

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &Hj_in, MatrixXd &R_in,
                        MatrixXd &R_ekf_in) {
  x_     = x_in;
  P_     = P_in;
  F_     = F_in;
  H_     = H_in;
  Hj_    = Hj_in;
  R_     = R_in;
  R_ekf_ = R_ekf_in;
}

void KalmanFilter::Predict() {
  // predict the state
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // update the state by using Kalman Filter equations
  VectorXd z_pred  = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size,x_size);
  P_ = (I - K* H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // Calculate measurement function h(x)
  float rho = sqrt(x_[0]*x_[0] + x_[1]*x_[1]);
  float phi = atan2(x_[1],x_[0]);
  float rhod = (x_[0]*x_[2] + x_[1]*x_[3]) / rho;

  VectorXd h(3);
  h << rho, phi, rhod;

  // Use jacobian and measurement function for EKF
  VectorXd y = z - h;

  // Normalize y[1] between pi and -pi
  do{
      if (y[1] > PI){
          y[1] -= PI;
      }
      else if (y[1] < -PI) {
          y[1] += PI;
      }
  } while (y[1] < -PI ||  y[1] > PI);

  MatrixXd Hjt = Hj_.transpose();
  MatrixXd S = Hj_ * P_ * Hjt + R_ekf_;
  //MatrixXd S = Hj_ * P_ * Hjt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Hjt;
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size,x_size);
  P_ = (I - K* Hj_) * P_;

}
