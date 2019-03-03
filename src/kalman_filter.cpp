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
   * TODO: predict the state
   */
  x_ = F_*x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  //This is the update step for the measurement data from the LIDAR, done using the standard Kalman Filter equations
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  //MatrixXd PHt = P_ * Ht;
  MatrixXd K = P_ * Ht * Si;

  //new estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  //Update step for measurement data from the RADAR, done using the Extended Kalman Filter equations
  
  double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  double phi = atan2(x_(1), x_(0));
  double rho_dot;
  
  if (fabs(rho) < 0.0001) 
  {
    rho_dot = 0;
  } 
  else 
  {
    rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
  }
  

  //rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;
  
  while (y(1)>(M_PI))
  {
    y(1) -= 2*M_PI;
  }
  while (y(1)<(-M_PI))
  {
    y(1) += 2*M_PI;
  }
  
//Below is another method I tried suggested by a friend of mine
  
//   while (y(1)>(M_PI/2)||y(1)<-(M_PI/2))
//   {
//     if (y(1)>(M_PI/2))
//       y(1)=-(M_PI);
//     else if (y(1)<-(M_PI/2))
//       y(1)=+(M_PI);
//   }
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  //MatrixXd PHt = ;
  MatrixXd K = P_ * Ht * Si;

  //new estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

//I foundthe mistake that I was doing in the timestep I was dividing by 10^5 instead of 10^6, I know I am an idiot 
//haaaaaaaaaaaaaaa...... The idiot that I have been, a true idiot, I truly apologize for cause such a mayhem both of my 
//mistakes were silly (I am quite known for that), I am very very sorry