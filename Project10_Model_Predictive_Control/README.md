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

## Implementation
---
The state vector includes the x and y coorid

| Number of Time Steps (N)         		|     Time Step Size	     | Result |
|:---------------------:|:---------------------------------------------:|:---------------------------------------------:|
| 20        			| 0.1   										| Final implementation parameters. The car navigates well.  |
| 20				     	| 0.2									 	| MPC predicts way into future. Car is able to navigate along the track but wheels pop out of the track at certian points.  |
| 20					| 0.05										| The car cannot navigate along the track successfully |

## Result
---
The final implementation of the MPC algorithm with a 100 millisecond latency can be found [here](https://www.youtube.com/watch?v=vSfGQtFNSi4). The yellow line denotes the waypoints or center of the track. The X and Y coordinates of the waypoints can found in lake_track_waypoints.csv. The green line indicates the trajectory predicted by the MPC algorithm. 
