#include "kalman_filter.h"

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

  VectorXd F_transpose_ = F_.transpose();
  P_ = F_ * P_ * F_transpose_ + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * Update the state by using Kalman Filter equations
   */

  VectorXd y_, S_ , K_, H_transpose_, S_inverted_, F_transpose_;
  MatrixXd I;

  H_transpose_ = H_.transpose();
  F_transpose_ = F_.transpose();

  y_ = z - H_ * x_;
  S_ = (H_ * P_) * H_transpose_ + R_;

  S_inverted_ = S_.inverse();

  K_ = (P_ * H_transpose_) * S_inverted_;
  x_ = x_ + K_ * y_;

  long x_size = x_size;
  I = MatrixXd::Identity(x_size, x_size);

  P_ = (I - K_ * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}
