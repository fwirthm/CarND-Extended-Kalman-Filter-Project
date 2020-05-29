#include "kalman_filter.h"
#include <iostream>
#include <math.h>

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
   *predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  //std::cout<<"prediction succesfull"<<std::endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   *update the state by using Kalman Filter equations
   */
   VectorXd z_pred = H_ * x_;
   VectorXd y = z - z_pred;
   MatrixXd Ht = H_.transpose();
   MatrixXd S = H_ * P_ * Ht + R_;
   MatrixXd Si = S.inverse();
   MatrixXd PHt = P_ * Ht;
   MatrixXd K = PHt * Si;

   //new estimate
   x_ = x_ + (K * y);
   long x_size = x_.size();
   MatrixXd I = MatrixXd::Identity(x_size, x_size);
   P_ = (I - K * H_) * P_;
   //std::cout<<"update (lidar) succesfull"<<std::endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   *update the state by using Extended Kalman Filter equations
   */
  
   double p_x = x_[0];
   double p_y = x_[1];
   double v_x = x_[2];
   double v_y = x_[3];
  
   double rho = sqrt(pow(p_x, 2) + pow(p_y, 2));
   double phi = atan2(p_y, p_x);
   double rho_dot = ((p_x*v_x)+(p_y*v_y))/rho;
   
   VectorXd h_x(3);
   h_x << rho, phi, rho_dot;

   VectorXd y = z - h_x;

    while(y(1) > M_PI){
        y(1) -= 2 * M_PI;
    }

    while(y(1) < -M_PI){
        y(1) += 2 * M_PI;
    }
  
   MatrixXd Ht = H_.transpose();
   MatrixXd S = H_ * P_ * Ht + R_;
   MatrixXd Si = S.inverse();
   MatrixXd PHt = P_ * Ht;
   MatrixXd K = PHt * Si;

   //new estimate
   x_ = x_ + (K * y);
   long x_size = x_.size();
   MatrixXd I = MatrixXd::Identity(x_size, x_size);
   P_ = (I - K * H_) * P_;
  
  //std::cout<<"update (radar) succesfull"<<std::endl;
}
