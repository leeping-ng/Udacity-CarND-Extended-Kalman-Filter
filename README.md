# Udacity-CarND-Extended-Kalman-Filter
An Extended Kalman Filter that tracks a bicycle's position and velocity using LiDAR and radar measurements, for the Udacity Self-Driving Car Engineer Nanodegree.

### 1. What is a Kalman Filter?

A Kalman Filter (KF) is useful in predicting what will happen to a dynamic system, when there is uncertain information about it.
- For example, in this project, by assuming that the bicycle speed is constant, the model will predict the location of the bicycle after a time interval
- However, there is uncertainty in this as the bicycle may accelerate or decelerate
- Next, the model will receive the measured location of the bicycle from LiDAR and radar measurements
- Again, there is uncertainty here as sensor measurements are not perfect

The KF is able to combine such uncertain information and estimate a more accurate result, as depicted by this graph from MathWorks, while at the same time being fast and light on memory.

<img src='https://github.com/leeping-ng/Udacity-CarND-Extended-Kalman-Filter/blob/master/writeup_images/KF%20Gaussians.jpg' width=500>

For more information, Bzarg wrote a good article explaining how a KF works: http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/

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

<img src='https://github.com/leeping-ng/Udacity-CarND-Extended-Kalman-Filter/blob/master/writeup_images/Udacity%20KF%20Equations.JPG' width=800>

### 3. Types of Kalman Filters

This section will discuss 3 different types of filters, namely:
- Kalman Filter (KF)
- Extended Kalman Filter (EKF)
- Unscented Kalman Filter (UKF)

The KF is able to model linear systems accurately, such as predicting the motion of an object with constant speed and direction, and taking measurements in Cartesian coordinates such as from a LiDAR.

When it comes to non-linear systems, either a EKF or UKF will need to be applied. For instance, in this project, radar measurements are also provided.
Radar measurements are in polar coordinates, and once they are mapped into Cartesian coordinates, they become non-linear. This non-linear transformation causes a Gaussian distribution to turn into a non-Gaussian distribution, and the standard KF equations can no longer be applied.

To be continued...


