## Project: Extended Kalman Filter
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Overview
---
In this project, we implement the extended Kalman filter in C++ using provided LIDAR and RADAR measurements of a bicycle that travels around our car. The objectives of the project can be summarized as below:

* Read the LIDAR and RADAR measurements
* Implement the standard Kalman filter for LIDAR measurements and extended Kalman filter for RADAR measurements 
* Compare the predicted path with the ground truth results and calculate the Root Mean Square Error (RMSE)
* RMSE Error should be less than 0.11 for the x and y values


[//]: # (Image References)

[image1]: ./write_up_images/snapshot_simulator.png "snapshot_simulator"

## Dependencies
---
* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Instructions to Run
---
* Clone this repo.
* Make a build directory: mkdir build && cd build
* Compile: cmake .. && make
* Run Simulator: Clone Term 2 Simulator (https://github.com/udacity/self-driving-car-sim/releases/) and run it
* Run it: ./ExtendedKF 

## Result
---
Currently the results are reported only for Dataset 1
The RMSE values for X, Y, VX and VY are always less than 0.11, 0.11, 0.52 and 0.52 respectively along the entire length of the path. 
The final values of RMSE are:

| Quantity         		|     RMSE	        							|
|:---------------------:|:---------------------------------------------:|
| X         			| 0.0974   										|
| Y				     	| 0.0855									 	|
| VX					| 0.4517										|
| VY	      			| 0.4404 										|

![alt text][image1]
