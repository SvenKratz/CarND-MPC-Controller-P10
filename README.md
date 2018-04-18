# MPC Controller Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This repository contains my solution to the Udacity Self-Driving Car NanoDegree MPC Controller Project.

The repository contains the following notable files:

* `src/MPC.cpp` : Implementation of MPC controller for the project
* `src/main.cpp` : Implementation of MPC controller `main()` function for the project
* this readme file

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## The Model

In the following I will discuss some key aspects of the MPC model, as required by the project rubric.

### Update Equations
I use the basic model update equations given in Lesson 20:

![Model Equations](images/model.png)

It models car movement, acceleration and turning and also incorporates CTE and orientation error.

### Actuators
Actuators are turning and acceleration, the two functions that the car provides.

Steering is constrained to the car's max steering angle of +- 0.436 radians. Acceleration to the car's max acceleration of +- 1.0 m/s.

### Timestep Length and dt

I chose N = 10 as a reasonable look-ahead length. With dt set to 100ms (which is also the average system latency as given by the project), this results in pre-planning the next second of vehicle motion, which seems to be a realistic extend of planning future actions.

### Waypoint Preprocessing

I preprocess the waypoint information by transforming waypoints to be relative to the vehicle (which is required for visualization anyway). This has the advantage of simplifying some model equations, as the initial state for x,y,phi can be set to 0, since waypoints are relative to the vehicle's frame of reference.

The following code performs the coordinate transform:

```
// transform points to car's refernece frame (x,y,phi) == (0,0,0)
for (int i = 0; i < ptsx.size(); i++) {
  double dx = ptsx[i] - px;
  double dy = ptsy[i] - py;
  tX.push_back(dx * cos(-psi) - dy * sin(-psi));
  tY.push_back(dx * sin(-psi) + dy * cos(-psi));
}
```

### Latency

I adjust the actuations by looking at calculating a weighted sum of the state from t-2 and a weighted actuation difference of states t-2 and t-1 during constraint setup. The following code deals with latency in my MPC controller (line 112):

```
// take previous actuations into account to mitigate latency
if (t > 1) {   
  a = 0.8 * vars[a_start + t - 2] +  0.2 * (vars[a_start + t - 2] - vars[a_start + t - 1]);
  delta = 0.8 * vars[delta_start + t - 2] + 0.2 * (vars[delta_start + t - 2] - vars[delta_start + t - 1])  ;
}
```
