#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  
  const int n = estimations.size();
  
  if (n == 0 || n != ground_truth.size()) {
    throw std::invalid_argument("RMSE: estimations size null or not matching ground_truth");
  }
  
  VectorXd rmse = VectorXd::Zero(estimations[0].size());
  for (size_t i = 0; i < n; i++) {
    rmse = rmse.array() + (estimations[i] - ground_truth[i]).array().square();
  }
  rmse = (rmse / n).cwiseSqrt();
  return rmse;
}
