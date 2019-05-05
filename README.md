# Udacity-CarND-Extended-Kalman-Filter
An Extended Kalman Filter that tracks a bicycle's position and velocity using LiDAR and radar measurements, for the Udacity Self-Driving Car Engineer Nanodegree.

### 1. What is a Kalman Filter?

A Kalman Filter (KF) is useful in predicting what will happen to a dynamic system, when there is uncertain information about it.
- For example, in this project, by assuming that the bicycle speed is constant, the model will predict the location of the bicycle after a time interval
- However, there is uncertainty in this as the bicycle may accelerate or decelerate
- Next, the model will receive the measured location of the bicycle from LiDAR and radar measurements
- Again, there is uncertainty here as sensor measurements are not perfect

The KF is able to combine such uncertain information and estimate a more accurate result, as depicted by this graph from MathWorks, while at the same time being fast and light on memory.

<img src='https://github.com/leeping-ng/Udacity-CarND-Extended-Kalman-Filter/blob/master/writeup_images/KF%20Gaussians%20-%20Mathworks.jpg' width=500>

For more information, here is good article by Bzarg explaining how a KF works: http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/

### 2. Kalman Filter Equations and Flow

The KF algorithm goes through the following steps, given by the KF equations in the image by Udacity below.
#### A. Initialisation 
- Receive initial measurements of the bicycle's position, from either the LiDAR or radar, and update *x*, which is the state vector of 2D positions and velocities
- Initialise the state covariance matrix, *P*, based on the first measurement

#### B. Predict and Update Loop
##### 1) Predict
- The KF algorithm will predict *x'*, the position of the bicycle after time *dt*, by assuming that the velocity of the bicycle is constant
- In reality, the bicycle may accelerate or decelerate by an unknown amount, and this is added as noise in *Q*, the process noise covariance matrix, in equation (12)
- The uncertainty of the prediction, *P'*, is also calculated

##### 2) Update
- The KF algorithm compares the predicted location with the actual sensor measurements, *z* in equation (13)
- In reality, the sensor measurement has uncertainty, and this is introduced in *R*, the measurement noise covariance matrix
- K is the Kalman Filter gain, which combines the uncertainty of the prediction P' and uncertainty of measurement R in equations (14) and (15)
- In equation (16), if the measurement uncertainty of the sensor R is high relative to prediction uncertainty P', more weight is given to the prediction x', and vice versa.
- An updated location, *x*, and updated prediction covariance matrix, *p*, are updated.

The KF algorithm will receive another sensor measurement after a time period dt, and another "predict" and "update" is done. This continues as a loop.

<img src='https://github.com/leeping-ng/Udacity-CarND-Extended-Kalman-Filter/blob/master/writeup_images/KF%20Equations%20-%20Udacity.JPG' width=800>

### 3. Types of Kalman Filters

This section will discuss 3 different types of filters, namely:
- Kalman Filter (KF)
- Extended Kalman Filter (EKF)
- Unscented Kalman Filter (UKF)

The KF is able to model linear systems accurately, such as predicting the motion of an object with constant speed and direction, and taking measurements in Cartesian coordinates such as from a LiDAR.

However, when it comes to non-linear systems, either a EKF or UKF will need to be applied. 

#### A. Extended Kalman Filter (EKF)

There are 4 differences between the KF and the EKF equations, which are:
- #1. Measurement Update: H matrix replaced by Jacobian matrix Hj when calculating S, K and P
- #2. Measurement Update: To calculate y in equation (13), h(x') will be used instead of Hx'
- #3. Prediction: F matrix replaced by Jacobian matrix Fj
- #4. Prediction: To calculate x' in equation (11), prediction update function f will be used instead of Fx

In this project, radar measurements are used in addition to LiDAR measurements. 
Radar measurements are in polar coordinates as follows:
- Rho: radial distance from origin to object
- Phi: angle between ray and x direction
- Rho_dot: radial/Doppler velocity along radial distance

Once they are mapped into Cartesian coordinates, they become non-linear. This non-linear transformation causes a Gaussian distribution to turn into a non-Gaussian distribution, and the standard KF equations can no longer be applied. Hence, changes #1 and #2 above are applied to the standard KF equations for this project.

Change #1: For the radar update step, there is no H matrix that can map the state vector x into polar coordinates. The mapping needs to be done manually to convert from Cartesian to polar coordinates, as shown in h(x’) below. Hence for radar, y = z – Hx’ becomes y = z – h(x’) instead.

<img src='https://github.com/leeping-ng/Udacity-CarND-Extended-Kalman-Filter/blob/master/writeup_images/h%20function.JPG' width=300>

Change #2: Using Taylor series expansion, a linear approximation of h(x’) is used to keep the distribution Gaussian. The Jacobian matrix Hj below is obtained from calculating partial derivatives and this will be used in place of the standard H matrix for LiDAR. 

<img src='https://github.com/leeping-ng/Udacity-CarND-Extended-Kalman-Filter/blob/master/writeup_images/Jacobian%20Matrix%20Hj.JPG' width=500>

As for changes #3 and #4 to predict motion, in this project, a simple, linear Constant Velocity (CV) model is used. In reality, the bicycle may turn or accelerate, and to model this more accurately, the following models can be used:
- constant turn rate and velocity magnitude model (CTRV)
- constant turn rate and acceleration (CTRA)
- constant steering angle and velocity (CSAV)
- constant curvature and acceleration (CCA)

#### B. Unscented Kalman Filter (UKF)

A key step of the EKF is to use a Taylor series expansion about a chosen point and create a Jacobian matrix to linearise the function and keep it Gaussian. 

For the UKF, instead of linearising about one operating point, a few sample points, called sigma points, are taken to create a better estimate. The computationally intensive Jacobian matrix does not need to be calculated for the UKF.


### 4. Project Code

The image below shows the simulator results of this project's EKF. In the image below, LiDAR measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers from the EKF are green triangles. Simulated LiDAR and radar data are provided to the C++ script, and the script feeds back to the simulator the measured estimation marker and Root Mean Square Error (RMSE) values.

<img src='https://github.com/leeping-ng/Udacity-CarND-Extended-Kalman-Filter/blob/master/writeup_images/RMSE%20Simulator.JPG' width=600>

#### Main.cpp

Main.cpp reads in the data from the simulator,  calls a function to run the Kalman filter, and calls a function to calculate RMSE
- creating an instance of the FusionEKF class
- Receiving the measurement data calling the ProcessMeasurement() function. ProcessMeasurement() is responsible for the initialization of the Kalman filter as well as calling the prediction and update steps of the Kalman filter. 

The rest of main.cpp will output the following results to the simulator:
-	estimation position
-	calculated RMSE

#### FusionEKF.cpp

-	initializes the filter, calls the predict function, calls the update function
-	FusionEKF.cpp has a variable called ekf_, which is an instance of a KalmanFilter class. The ekf_ will hold the matrix and vector values. You will also use the ekf_ instance to call the predict and update equations.

#### kalman_filter.cpp
- defines the predict function, the update function for lidar, and the update function for radar
