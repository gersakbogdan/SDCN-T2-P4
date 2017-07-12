# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

or

1. Build and run: `./run.sh`


## Describe the effect each of the P, I, D components had in your implementation.

* #### P component
The Proportional term produces an output value proportional to the CTE value.
P term can be adjusted by multiplying the error with a constant value Kp called the proportional gain constant.
Using a small value for Kp parameter will result in a small change in the output (steering angle)
and using a high value will result in a larger output response. However if Kp parameter is too high the system
can start to oscillate around the target and can become unstable.


* #### I component
The I term is proportional to both the magnitude of the error and the duration of the error.
The I term accelerates the movement of the process towards setpoint and eliminates the residual steady-state error that occurs with a pure P controller.

* #### D component
The D term is proportional to the rate of change of CTE.
The D term improves settling time and stability of the system and reduces the overshooting and oscilattions caused by P term.


## Describe how the final hyperparameters were chosen.
PID Controller must be tuned in order to suit with dynamics of the process to be controlled.
There are different type of tuning methods which requires much attention to select the best values for P I D gains.
For this project is used the Trial and Error Method.

Trial and Error Method is a simple method of PID controller tuning. We can tune the controller by first setting the
Ki and Kd values to zero and increase P term until system reaches the oscillating behavior. Once it is oscillating
Ki term needs to be adjusted to stop the oscilattion and finally D term is adjusted to get faster response.


