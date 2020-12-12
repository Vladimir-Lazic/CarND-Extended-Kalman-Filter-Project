#include "kalman_filter.h"

#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * Kalman filter prediction step
   */

  x_ = F_ * x_;

  MatrixXd Fi = F_.transpose();
  P_ = F_ * P_ * Fi + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * Update the state by using Kalman Filter equations
   */

  VectorXd y_;
  MatrixXd S_, K_, Ht, Si, Fi, I;

  Ht = H_.transpose();
  Fi = F_.transpose();

  y_ = z - H_ * x_;
  S_ = (H_ * P_) * Ht + R_;

  Si = S_.inverse();

  K_ = (P_ * Ht) * Si;
  x_ = x_ + K_ * y_;

  long x_size = x_.size();
  I = MatrixXd::Identity(x_size, x_size);

  P_ = (I - K_ * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  float rho, phi, rho_dot;
  VectorXd h, y;
  MatrixXd Ht, S, Si, K, I;

  // Initializing values
  h = VectorXd(3);

  // Linearize the non linear prediction and measurement funcitons
  rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  phi = atan2(x_(1), x_(0));

  if (fabs(rho) < 0.0001) {
    rho_dot = 0;
  } else {
    rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;
  }

  // Use the same Kalman filter mechanism to estimate the new state
  h << rho, phi, rho_dot;

  y = z - h;

  while (y(1) > M_PI || y(1) < -M_PI) {
    if (y(1) > M_PI) {
      y(1) -= M_PI;
    } else {
      y(1) += M_PI;
    }
  }

  Ht = H_.transpose();

  S = H_ * P_ * Ht + R_;

  Si = S.inverse();

  K = P_ * Ht * Si;

  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
