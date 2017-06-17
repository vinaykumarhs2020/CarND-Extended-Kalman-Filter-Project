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
  VectorXd _diff(4);
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
}
