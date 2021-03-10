#include "kalman_filter.h"
#include "tools.h"
#include<iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */
#define PI 3.14159265
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
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_=x_+K*(z- H_*x_);
  int rows= P_.rows();
  MatrixXd I;
  I = MatrixXd::Identity(rows,rows);
  P_ = (I- K*H_)*P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  
  float px= x_(0);
  float py= x_(1);
  float vx= x_(2);
  float vy= x_(3);
  
  
  float rho=sqrt(px*px+py*py);
  if(rho<0.0001){
    std::cout<<"Range approaching zero, set to 0.0001";
    rho=0.001;
  }
  float theta= atan2(py,px);
  
  VectorXd h(3);
  h<< rho, theta, (px*vx + py*vy)/rho;
  VectorXd y=z-h;
  if(y(1)<-PI){std::cout<<"PI is incremented"<<std::endl;y(1)=y(1)+2*PI;}
  if(y(1)>PI){std::cout<<"PI is decremented"<<std::endl;y(1)=y(1)-2*PI;}
  std::cout<<"Value of y(1) is:"<<y(1)<<std::endl;
  MatrixXd S = H_ * P_ * H_.transpose() +R_;
  MatrixXd K = P_ *H_.transpose() * S.inverse();
  int rows = P_.rows();
  MatrixXd I;
  I = MatrixXd::Identity(rows,rows);
  P_ = (I-K*H_)*P_;
  x_ = x_ + K*(y);
  
  
}
