# Extended Kalman Filter Project (Sensor Fusion)

## 1.0 Make & Run Project (Linux)
Download the simulator for term 2 [HERE](https://github.com/udacity/self-driving-car-sim/releases/). From the main repository folder compile the code using make and run the project to ensure there were no build errors.

	$ chmod +x install-ubuntu.sh`
	$ /install-unbuntu.sh
	$ mkdir build && cd build
	$ cmake .. && make
	
If there are no build errors then run the simulator (client) and the Extended Kalman Filter (server) file from the build folder in the project repository. The two programs talk to one another through the uWebSockets on local port 4567.

	$ # From the simulator folder.
	$ ./term2_sim.x86_64
	$ # From the project folder.
	$ ./build/ExtendedKF

From the terminal you should see "Listening to port 4567" until you connect and start the Udacity Simulator. If the connection is successful you should see "Connected!!!" followed by a print out of values.

## 2.0 Complete the Project Classes
### 2.1 Tools.cpp
#### 2.1.1 Calculate Root Mean Squared Error (RMSE)
The Tools::CalculateRMSE() function takes two vectors, estimation and ground truth, as inputs and returns a vector of RMSE. For every entry in the vector a residual squared sum is totalled and then divided my the length of the vector. This is a parameter used for determining the quality of the algorithm on tracking the models position as it moves.

	$ for (int i=0; i < estimations.size(); i++) {
	$	VectorXd residual = estimations[i] - ground_truth[i];
	$	residual = residual.array()*residual.array();
	$	rmse += residual;
	$ }

#### 2.1.2 Calculate Jacobian
The Tools::CalculateJacobian() function is used for converting radar measurements from polar to cartesian coordinates and takes in a vector defining the x_state. It is important to check for division by zero as to avoid a calculation error.

	$ // Extract kinematic terms from matrix.
	$ MatrixXd Hj(3,4);
	$ MatrixXd Hj(3,4);
	$ float px = x_state(0);
	$ float py = x_state(1);
	$ float vx = x_state(2);
	$ float vy = x_state(3);

	$ // Pre-compute constants for human readability.
	$ float c1 = px*px+py*py;
	$ float c2 = sqrt(c1);
	$ float c3 = (c1*c2);

	$ // Check for division by zero.
	$ if(fabs(c1) < 0.0001) {
	$ 	cout << "Tools::CalculateJacobian () - Error - Division by Zero" << endl;
	$ 	return Hj;
	$ }


### 2.2 FusionEKF.
The FusionEKF class facilitates the input from multiple sources such as laser and radar readings and feeds them into the Kalman filter accordingly. The class constructor initializes the specific matrices for handling the laser and radar measurements during the initialization, prediction, and update steps.

#### 2.2.1 Initialization
Upon the first measurement input, the class sets the initial reading as the current state, updates the previous time stamp and then returns without performing the predction and update steps.

**Note:** If previous_timestamp_ is not set here an error will be thrown if a radar measurement is received first.

#### 2.2.2 Prediction
With a MeasurementPackage object as the input, the Kalman Filter parameters (state transition & process covariance) are updated based on the time difference between samples.

#### 2.2.3 Update
...

### 2.3 Kalman_Filter.cpp
...


## 3.0 Running the Simulator
### 3.1 Laser Data Only
Running the simulator with only laser data by ignoring data samples that are labelled as "RADAR". The resulting estimate of position and velocity is decent but the accuracy of the values are not enough to satisfy this project with average RMSE for X/Y/VX/VY were 0.1840, 0.1543, 0.6056, 0.4861 respectively.

<p align="center">
 <img src="./images/data1_laser.gif" width=550>
</p>

### 3.2 Radar Data Only
Running the simulator with only radar data by ignoring data samples that are labelled as "LASER". The resulting estimate of position and velocity is decent but the accuracy of the values are not enough to satisfy this project with average RMSE for X/Y/VX/VY were 0.2318, 0.2985, 0.5943, 0.5848 respectively.

<p align="center">
 <img src="./images/data1_radar.gif" width=550>
</p>

### 3.3 Sensor Fusion (Laser & Data)
Running the simulator with both radar and laser data results in a much more accurate estimate of position for both position and velocity. The average RMSE for X/Y/VX/VY were 0.0974, 0.0855, 0.4517, 0.4404 respectively which satisfy the requirements for the project (0.11, 0.11, 0.52, 0.52).

<p align="center">
 <img src="./images/data1_all.gif" width=550>
</p>
