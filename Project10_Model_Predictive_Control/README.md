## Project: Model Predictive Control
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Overview
---
In this project, we design and implement a Model Predictive Control (MPC) algorithm to help navigate a car around a race track given the waypoints of the track.

[//]: # (Image References)

[image1]: ./write_up_images/success.png "success"

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
* Run Code: ./mpc

## Result
---
The number of particles used are 50. With 50 particles the errors in x, y and yaw are below the required levels. On running ./run.sh in the terminal, the output "Success!Your particle filter passed!" is obtained.
![alt text][image1]