## Project: PID Controller
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Overview
---
In this project, we design and implement a proportional-integral-derivative (PID) controller to help navigate a car around a race track given the cross track error.

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
* Run Code: ./pid

## Result
---
The video of the final implementation of the PID controller can be found [here](https://www.youtube.com/watch?v=t1KleOY2bzE&t=6s).
The selection of the final hyperparameters i.e. coefficients of the proportional term Kp, integral term Ki and derivative term Kd is done manually by trial and error method or manual tuning. The final values of the hyperparameters are Kp = -0.12, Ki = 0.0, Kd = -2.6. A more efficient approach to determine these hyperparameters dynamically would be to use an optimization algorithm like twiddle. The advantage of using an optimization algorithm will be evaluated in the furture and is beyond the scope of this work.

## Effect of each of the P, I, D components
---

### Effect of P component:
In order study the effect of the P component of the PID algorithm, the value of Kp is doubled (Kp = -0.24) and the values of Ki and Kd are maintained constant. [Here](https://www.youtube.com/watch?v=zkT11jwjtyM&t=1s) is the video with Kp = -0.24, Ki = 0.0, Kp -2.6. We can distinctly see that the car oscillates significantly more when the Kp is doubled.

### Effect of I component:
To investigate the effect of the I component of the PID algorithm, a non-zero value of Ki is used (Ki = -0.006) while keeping the values of Kp and Kd same as the final implementation (Kp = -0.12, Kd = -2.6). [Here](https://www.youtube.com/watch?v=R58Aq7JLbxU) is the video with non-zero value of Ki. In the beginning we can see that the car tends to oscillate more in comparison to the final implementation. This is because of accumulation of the cross track error. For even lower values of Ki, the car goes of track immediately when the simulator is run.

### Effect of D component:
The effect of the D component of the PID algorithm is evaluated by scaling the value of Kd in the final implementation so that the new value is Kd = -5.2 and the values of Kp and Ki are maintained constant. [Here](https://www.youtube.com/watch?v=5V6CX82JDbo) is the video with Kp = -0.12, Ki = 0.0, Kp -5.2. For this case we see that the car has a tendency to move away from the center of the track especially near the corners. This is because the steering angle appears to be dominated by the derivative of the cross track error and not the magnitude of the cross track error. A very small value of Kd will lead to significant oscillation.




