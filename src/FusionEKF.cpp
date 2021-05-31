#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;


  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  R_laser_ << 0.0225, 0,
              0, 0.0225;

  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  H_laser_<<1, 0, 0, 0,
  			0, 1, 0, 0;
  ekf_.P_ = MatrixXd(4,4);
  ekf_.F_=MatrixXd(4,4);
  ekf_.Q_=MatrixXd(4,4);


}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  cout<<"Call to Process Measurements"<<endl;
  if (!is_initialized_) {

    // first measurement
    cout << "EKF first measurement: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 0.5, 0.5;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

      float rho = measurement_pack.raw_measurements_(0);
      float theta = measurement_pack.raw_measurements_(1);

		ekf_.x_(0)=rho*cos(theta);
		ekf_.x_(1)=rho*sin(theta);
      cout<<"Inside RADAR";

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

      float px = measurement_pack.raw_measurements_(0);
      float py = measurement_pack.raw_measurements_(1);

		ekf_.x_(0)=px;
		ekf_.x_(1)=py;
     cout<<endl<<"Inside LIDAR"<<endl;
    }

    previous_timestamp_=measurement_pack.timestamp_;
    is_initialized_ = true;
    ekf_.P_ << 1, 0, 0, 0,
    			0, 1,0,0,
    			0,0,500,0,
    			0,0,0,500;
    ekf_.F_<<1,0,1,0,
    		0,1,0,1,
    		0,0,1,0,
    		0,0,0,1;
    cout<<endl<<"Initialisation done!!!"<<endl;
    return;
  }

  /**
   * Prediction
   */

  float dt= (measurement_pack.timestamp_-previous_timestamp_)/1000000.0;

   ekf_.F_(0,2)=dt;
  ekf_.F_(1,3)=dt;
  float ax=9, ay=9;
    previous_timestamp_=measurement_pack.timestamp_;
  ekf_.Q_ << pow(dt,4)*ax/4, 0, pow(dt,3)*ax/2,0,
  			0, pow(dt,4)*ay/4, 0, pow(dt,3)*ay/2,
  			pow(dt,3)*ax/2, 0 , pow(dt,2)*ax, 0,
            0, pow(dt,3)*ay/2, 0 , pow(dt,2)*ay;
  cout<<"Just before prediction"<<endl;
  ekf_.Predict();
  cout<<"Prediction done!"<<endl;
  /**
   * Update
   */


  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.R_=MatrixXd(3,3);
    ekf_.R_=R_radar_;
    Hj_=tools.CalculateJacobian(ekf_.x_);
    ekf_.H_=Hj_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    cout<<"EKF Updation done"<<endl;

  } else {
    ekf_.R_=MatrixXd(2,2);
    ekf_.R_=R_laser_;
    ekf_.H_=H_laser_;
	ekf_.Update(measurement_pack.raw_measurements_);
    cout<<"KF lidar updation done"<<endl;
  }

  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
