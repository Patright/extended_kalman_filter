#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

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
  Hj_ = MatrixXd(3, 4);

  // Measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  // Measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  // Initalize the F, P & Q matrices. 
  ekf_.F_ = MatrixXd(4,4);
  ekf_.P_ = MatrixXd(4,4);
  ekf_.Q_ = MatrixXd(4,4);
  
  // Assign the values for the laser sensor matrix, H_laser
  H_laser_ << 1,0,0,0,
	          0,1,0,0;
  
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*
   *  Initialization
   */
  if (!is_initialized_) {
   
    // first measurement
    cout << "EKF: \n";
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      
      ekf_.x_(0) = rho*cos(phi);
      ekf_.x_(1) = rho*sin(phi);
       
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];
      
      ekf_.x_(0) = px;
      ekf_.x_(1) = py;
    }
    // Capture the timestamp
    previous_timestamp_ = measurement_pack.timestamp_;
    // assign initial values to the state transition matrix
    ekf_.F_ << 1,0,1,0,
	           0,1,0,1,
	           0,0,1,0,
	           0,0,0,1;    
    // assign initial values to the covariance matrix
    ekf_.P_ << 1,0,0,0,
	           0,1,0,0,
	           0,0,500,0,
	           0,0,0,500;    

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*
   *  Prediction
   */
  
  // Calculate deltaT
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  float dt2 = dt*dt;
  float dt3 = dt2*dt;
  float dt4 = dt3*dt;
  // Set the values for acceleration noise
  float noise_ax = 9;
  float noise_ay = 9; 

  // Save the current timestamp for use in the next predict cycle
  previous_timestamp_ = measurement_pack.timestamp_;  

  // Update the state transition matrix
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  // Set the process covariance matrix
  ekf_.Q_ << dt4/4*noise_ax, 0, dt3/2*noise_ax, 0,
	         0, dt4/4*noise_ay, 0, dt3/2*noise_ay,
	         dt3/2*noise_ax, 0, dt2*noise_ax, 0,
	         0, dt3/2*noise_ay, 0, dt2*noise_ay;
  
  ekf_.Predict();

  /*
   *  Update
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // Calculate the Jacobian matrix
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    // Initialize the measurement covariance matrix
    ekf_.R_ = MatrixXd(3,3);
    ekf_.R_ = R_radar_; 
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    // Initialize the measurement covariance matrix
    ekf_.R_ = MatrixXd(2,2);
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}