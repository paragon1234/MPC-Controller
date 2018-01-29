# MPC-Controller

## 1. Objective
The main goal of the project is to implement in C++ Model Predictive Control (MPC) to drive the car around the track in a simulator. The program uses a simple Global Kinematic Model. The simulator provides reference waypoints (yellow line in the demo video) via websocket, and we use MPC to compute steering and throttle commands to drive the car. The solution must be robust to 100ms latency, since it might encounter in real-world application.

In this project, the MPC optimize the actuators (steering and throttle), simulate the vehicle trajactory, and minimize the cost like cross-track error.

## 2. Kinematic model

A kinematic model is implemented to control the vehicle around the track. Kinematic models are simplifications of dynamic models that ignore tire forces, gravity, and mass. This simplification reduces the accuracy of the models, but it also makes them more tractable. At low and moderate speeds, kinematic models often approximate the actual vehicle dynamics.

**States**: 

Position (_x,y_), heading (_ψ_) and velocity (_v_) form the vehicle state vector:

State: _[x,y,ψ,v]_

![State](readme_img/state.png)


**Actuator values**:

There are two actuators. Stearing angle (_δ_) is the first one, it should be in range [-25,25] deg. For simplicity the throttle and brake represented as a singular actuator (_a_), with negative values signifying braking and positive values signifying acceleration. It should be in range [-1,1].

Actuators: _[δ,a]_


**Update equations**:

The kinematic model can predict the state on the next time step (state at t+1) from the current state (state vector at t) and actuators values as follows:

![Kinematic model](readme_img/eq1.png)

where _Lf_ measures the distance between the front of the vehicle and its center of gravity. The parameter was provided by Udacity.


**Model Errors**:

Errors: cross track error (_cte_) and _ψ_ error (_eψ_) were used to build the cost function for the MPC. They could be updated on a new time step using the following equations:

![Erroers update model](readme_img/eq2.png)
