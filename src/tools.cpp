#include "tools.h"
#include <iostream>
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  std::cout<<"Inside RMSE Calculations"<<std::endl;
  VectorXd rmse(4);
  rmse<<0,0,0,0;
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }
  std::cout<<"Done delta calculation"<<std::endl;
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  float px= x_state(0);
  float py= x_state(1);
  float vx= x_state(2);
  float vy= x_state(3);
  
  float c1= pow((pow(px,2)+pow(py,2)),0.5);
  if(c1<0.0001){std::cout<<"rho is very small, setting to 0.0001";
               c1=0.0001;}
  float c2= pow(c1,2);
  float c3= pow(c1,3);
  
  MatrixXd Hj(3,4);
  Hj<< px/c1, py/c1, 0, 0,
  		-py/c2, px/c2, 0, 0,
  		py*(vx*py-px*vy)/c3, px*(vx*py-vy*px)/c3, px/c1, py/c1;
  
  return Hj;
  
}
