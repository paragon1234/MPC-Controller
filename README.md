# MPC-Controller

## Model

A simple Kinematic model (ignores tire forces, gravity, mass, etc) was used for the Controller. Some attempts to build more complicated dynamic model were made, but with low success. It is essential to know parameters of the vehicle (such as law of response on the throttle, geometry of the car, drag model, tires properties, etc) to construct a reasonable dynamic model but such parameters are not derectly accessible from provided materials for the project. 

Position (_x,y_), heading (_ψ_) and velocity (_v_) form the vehicle state vector:

State: _[x,y,ψ,v]_

![State](readme_img/state.png)

There are two actuators. Stearing angle (_δ_) is the first one, it should be in range [-25,25] deg. For simplicity the throttle and brake represented as a singular actuator (_a_), with negative values signifying braking and positive values signifying acceleration. It should be in range [-1,1].

Actuators: _[δ,a]_

The kinematic model can predict the state on the next time step by taking into account the current state and actuators as follows:

![Kinematic model](readme_img/eq1.png)

where _Lf_ measures the distance between the front of the vehicle and its center of gravity. The parameter was provided by Udacity.

Errors: cross track error (_cte_) and _ψ_ error (_eψ_) were used to build the cost function for the MPC. They could be updated on a new time step using the following equations:

![Erroers update model](readme_img/eq2.png)
