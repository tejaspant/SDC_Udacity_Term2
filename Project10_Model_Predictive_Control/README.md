## Project: Model Predictive Control
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Overview
---
In this project, we design and implement a Model Predictive Control (MPC) algorithm to help navigate a car around a race track given the waypoints of the track.

[//]: # (Image References)

[image1]: ./write_up_images/equations.png "equations"

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
The state vector includes the x and y coordinates of the car, the orientation angle psi, velocity, cross track error cte and error in the orientation angle epsi. The time stepping in the MPC algorithm is done using the following set of equations: 

![alt text][image1]

It should be noted that here that the x, y coordinates and psi are in the car coordinates. cte and epsi are calculated from polynomial fitting of the waypoints which are also converted into car coordinates. The actuators are the steering angle and throttle value.

As a part of the parametric study conducted for the MPC algorithm, we looked at the effect of changing the number of time steps N and time step size dt.
### Effect of Time Step Size (dt)

Here we maintain N and change dt.

| N          		|     dt (sec)    | Result |
|:---------------------:|:---------------------------------------------:|:---------------------------------------------:|
| 20        			| 0.1   										| Final implementation parameters. The car is able to successfully navigate around the track.  |
| 20				     	| 0.2									 	| Car is able to navigate along the track but wheels pop out of the track at certain points. This is because the MPC predicts signficantly into the future. |
| 20					| 0.05										| The car cannot successfully navigate around the track. |

### Effect of Number of Time Steps (N)

Here we keep dt constant and the see the effect of changing N.

|   dt (sec)    |     	 N     | Result |
|:---------------------:|:---------------------------------------------:|:---------------------------------------------:|
| 0.1        			| 10  										| Final implementation parameters. The car is able to successfully navigate around the track.  |
| 0.1				     	| 20									 	| The car cannot successfully navigate around the track.  |
| 0.1					| 5										| The car cannot successfully navigate around the track. |

## Result
---
The final implementation of the MPC algorithm with a 100 millisecond latency can be found [here](https://www.youtube.com/watch?v=vSfGQtFNSi4). The yellow line denotes the waypoints or center of the track. The X and Y coordinates of the waypoints can found in lake_track_waypoints.csv. The green line indicates the trajectory predicted by the MPC algorithm. 
