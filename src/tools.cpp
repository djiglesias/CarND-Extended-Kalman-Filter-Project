#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

	// Vector for rmse.
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// Check for divide by zero error.
	if(estimations.size() == 0) {
	    cout << "CalculateRSME () - Error - Zero length data" << endl;
	    return rmse;
	}
	if(estimations.size() != ground_truth.size()) {
	    cout << "CalculateRSME () - Error - Data lengths not equal" << endl;
	    return rmse;
	}
	
	// Calculate squared residuals.
	for (int i=0; i < estimations.size(); i++) {

		VectorXd residual = estimations[i] - ground_truth[i];

		residual = residual.array()*residual.array();
		rmse += residual;

	}

	// Calculate the sample mean.
	rmse = rmse/estimations.size();

	// Calculate the squared root.
	rmse = rmse.array().sqrt();

	// Return the result.
	return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

	// Extract kinematic terms from matrix.
	MatrixXd Hj(3,4);
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// Pre-compute constants for human readability.
	float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	// Check for division by zero.
	if(fabs(c1) < 0.0001) {
		cout << "Tools::CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

	// Compute the Jacobian Matrix.
	Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	// Return the result.
	return Hj;

}
