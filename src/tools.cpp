#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
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
  //std::cout << "Calculating Jacobian for:" << std::endl << x_state << std::endl;
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  float hypot = px*px + py*py;
  float root = sqrt(hypot);
  float hypot_root = hypot*root;

  // TODO: YOUR CODE HERE 
  // compute the Jacobian matrix
  // check division by zero
  if (fabs(hypot) < 0.0001) {
	Hj << 	1000, 1000, 0, 0,
     		1000, 1000, 0, 0,
    		1000, 1000, 1000, 1000;
    cout << "Measured location is (" << px << "," << py << ").  Difficult to calculate Jacobian.  Outputting high covariance so that predicted is trusted over measured." << endl << Hj << endl;
    return Hj;
  }
  
  
  Hj <<     px/root, py/root, 0, 0,
            -py/hypot, px/hypot, 0, 0,
            py*(vx*py - vy*px)/hypot_root, px*(vy*px - vx*py)/hypot_root, px/root, py/root;

  return Hj;
}

