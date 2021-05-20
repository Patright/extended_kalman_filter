#include <iostream>
#include <cmath>
#include "tools.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // Validate the estimations vector
  if(estimations.size() == 0 || estimations.size() != ground_truth.size())
  {
    cout<<"Error in size of Estimations vector or size mismatch with Ground Truth vector";
    return rmse;
  }
  
  // Accumulate the residual
  for(unsigned int i = 0; i < estimations.size(); ++i)
  {
  VectorXd residual = estimations[i] - ground_truth[i];
  rmse = rmse + (residual.array() * residual.array()).matrix();
  }

  //Mean and Sqrt the error
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  // Initalize the Jacobian
  MatrixXd Hj(3,4);
  // Get the state values
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Set the Jacobian to zero
  Hj << 0,0,0,0,
        0,0,0,0,
        0,0,0,0;

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = pow(px,2) + pow(py,2);
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // check division by zero
  if (fabs(c1) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
       -(py/c1), (px/c1), 0, 0,
       py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;


  return Hj;

}