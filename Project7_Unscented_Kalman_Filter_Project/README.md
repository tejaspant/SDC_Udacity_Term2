## Project: Unscented Kalman Filter
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Overview
---
In this project, we implement the unscented Kalman filter in C++ using provided LIDAR and RADAR measurements of a bicycle that travels around our car. The objectives of the project can be summarized as below:

* Read the LIDAR and RADAR measurements
* Implement the unscented Kalman filter for LIDAR measurements and extended Kalman filter for RADAR measurements 
* Experiment with process noise parameters $$\alpha$$
* Compare the predicted path with the ground truth results and calculate the Root Mean Square Error (RMSE)
* RMSE Error should be less than [0.09, 0.10, 0.4, 0.3] for the x location (X), y location (Y), x velocity (VX) and y velocity (VY) respectively


[//]: # (Image References)

[image1]: ./write_up_images/screenshot_sim.png "snapshot_simulator"
[image2]: ./write_up_images/nis_lidar_radar.png "nis plot"
[image3]: ./write_up_images/ukf_prediction.png "ukf_plot"

## Dependencies
---
* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4
* [Eigen library](http://eigen.tuxfamily.org/index.php?title=Main_Page) 

## Instructions to Run
---
* Clone this repo.
* Make a build directory: mkdir build && cd build
* Compile: cmake .. && make
* Run Simulator: Clone [Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases/) and run it
* Run Code: ./ExtendedKF 

## Result
---
Currently the results are reported only for Dataset 1
The RMSE values for X, Y, VX and VY are always less than 0.09, 0.10, 0.4 and 0.3 respectively along the entire length of the path. 
The final values of RMSE are:

| Quantity         		|     RMSE	        							|
|:---------------------:|:---------------------------------------------:|
| X         			| 0.0745   										|
| Y				     	| 0.0805									 	|
| VX					| 0.2767										|
| VY	      			| 0.2438 										|

![alt text][image1]
![alt text][image2]
![alt text][image3]
