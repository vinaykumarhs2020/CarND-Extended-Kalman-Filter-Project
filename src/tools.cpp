#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  VectorXd _diff(4);
  if(estimations.size() != ground_truth.size() || estimations.size() == 0){
    // std::cout << "Return 0 rmse" << '\n';
    return rmse;
  }
  for (size_t i = 0; i < estimations.size(); i++) {
    _diff =  ground_truth[i] - estimations[i]; // Diff
    _diff = _diff.array().square(); // Square
    rmse += _diff;
  }
  rmse = rmse / estimations.size(); // Mean
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  // Declare Hj:
  MatrixXd Hj(3,4);
  // Seperate the terms in state vector
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Calculate intermediate terms
  float c1 = px*px + py*py;
  // Check for division by zero:
  if (c1 < 0.00001){
    // std::cout << "Division by zero!!!" << std::endl;
    // return Hj;
    // Add a very small normalizing factor:
    c1 += 0.00001;
  }
  float c2 = sqrt(c1);
  float c3 = c1 * c2;

  Hj << (px/c2), (py/c2), 0, 0,
        -(py/c1), (px/c1), 0, 0,
        py*(vx*py-vy*py)/c3, px*(px*vy-vx*py)/c3, px/c2, py/c2;

  return Hj;
}
