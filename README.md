# Kalman Filter Project (Sensor Fusion)
# Term 2 - Project 1




# Project notes
## Install & Run
- from the main repo run 
- chmod +x install-ubuntu.sh
- ./install-unbuntu.sh
- mkdir build && cd build
- cmake .. && make
- ./ExtendedKF
- terminal should pop up.
- run the simmulator and select the correct channel/project
- you should see the "connected!"

## Tools.cpp

### Calculate RMSE.
	// Vector for rmse.
	VectorXd rmse(4);
	rsme << 0, 0, 0, 0;

	// Check for divide by zero error.
	if(estimations.size() == 0){
	    cout << "CalculateRSME () - Error - Zero length data" << endl;
	    return rmse;
	}
	if(estimations.size() != ground_truth.size()){
	    cout << "CalculateRSME () - Error - Data lengths not equal" << endl;
	    return rmse;
	}
	
	// Calculate squared residuals.
	for (int i=0; i < estimations.size(); i++) {

		VectorXd residual = estimations[i] - ground_truth[i];

		residual = residual.array()*residual()
		rmse += residual;

	}

	// Calculate the sample mean.
	rmse = rmse/estimations.size();

	// Calculate the squared root.
	rmse = rmse.array().sqrt();

	// Return the result.
	return rmse;

### Calculate Jacobian Matrix.

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
	if(fabs(c1) < 0.0001){
		cout << "Tools::CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

	// Compute the Jacobian Matrix.
	Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	// Return the result.
	return Hj;



## FusionEKF.cpp
- init function
-- 



## Kalman_Filter.cpp

must set time on initial measurement else OVF will occur with radar first


RMSE [X, Y, VX, VY]
Run #1 (Data 1): 0.0974, 0.0855, 0.4517, 0.4404 (ALL)
Run #2 (Data 2): 0.0726, 0.0965, 0.4219, 0.4937 (ALL)
Run #3 (Data 1): 0.1840, 0.1543, 0.6056, 0.4861 (LASER)
Run #4 (Data 2): 0.1650, 0.1561, 0.5888, 0.5068 (LASER)
Run #5 (Data 1): 0.2318, 0.2985, 0.5943, 0.5848 (RADAR)
Run #6 (Data 2): 0.2440, 0.3379, 0.6039, 0.8182 (RADAR)