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
  rmse << 0, 0, 0, 0;

  if (!estimations.size() || estimations.size() != ground_truth.size()) {
    cout<<"Estimation vector size should not be zero, or the should equal to ground truth!!!"<<endl;
    return rmse;
  }

  //accumulate squared residuals
	for (int i = 0; i < estimations.size(); ++i) {
    VectorXd residuals = estimations[i] - ground_truth[i];
    VectorXd squared_residuals = residuals.array() * residuals.array();
    rmse = rmse + squared_residuals;
	}
	//calculate the mean
  rmse = rmse / estimations.size();
	//calculate the squared root
	rmse = rmse.array().sqrt();
	//return the result
	return rmse;
}
