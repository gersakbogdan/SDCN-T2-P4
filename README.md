# PID Controller Project
Self-Driving Car Engineer Nanodegree Program

---

[//]: # (Image/Video References)

[image1]: ./output/P01.gif "Params = [0.1, 0, 0]"
[video1]: ./output/P01.mov "Params = [0.1, 0, 0]"
[image2]: ./output/P1.gif "Params = [1, 0, 0]"
[video2]: ./output/P1.mov "Params = [1, 0, 0]"
[image3]: ./output/P5.gif "Params = [5, 0, 0]"
[video3]: ./output/P5.mov "Params = [5, 0, 0]"
[image4]: ./output/P01-D1.gif "Params = [0.1, 1, 0]"
[video4]: ./output/P01-D1.mov "Params = [0.1, 1, 0]"
[image5]: ./output/P01-D5.gif "Params = [0.1, 5, 0]"
[video5]: ./output/P01-D5.mov "Params = [0.1, 5, 0]"
[image6]: ./output/P01-D10.gif "Params = [0.1, 10, 0]"
[video6]: ./output/P01-D10.mov "Params = [0.1, 10, 0]"

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

or

1. Build and run: `./run.sh`


## PID Controller

* #### P component
The Proportional term produces an output value proportional to the CTE value.
P term can be adjusted by multiplying the error with a constant value Kp called the proportional gain constant.
Using a small value for Kp parameter will result in a small change in the output (steering angle)
and using a high value will result in a larger output response. However if Kp parameter is too high the system
can start to oscillate around the target and can become unstable.

| P I D parameters | Result | Conclusion |
| --- | --- | --- |
| 5, 0, 0 | <img src="./output/P5.gif" width="320px" height="237px" /> | Definitely too high, vehicle is unstable.
| 1, 0, 0 | <img src="./output/P1.gif" width="320px" height="237px" /> | Still too high but a little bit better.
| 0.1, 0, 0 | <img src="./output/P01.gif" width="320px" height="237px" /> | Not bad, vehicle bounce from left to right but stays on track on a straight line. In order to have a faster response when turn D gain should be used.

* #### I component
The I term is proportional to both the magnitude of the error and the duration of the error.
The I term accelerates the movement of the process towards setpoint and eliminates the residual steady-state error that occurs with a pure P controller.

* #### D component
The D term is proportional to the rate of change of CTE.
The D term improves settling time and stability of the system and reduces the overshooting and oscilattions caused by P term.


## Hyperparameters Tuning
PID Controller must be tuned in order to suit with dynamics of the process to be controlled.
There are different type of tuning methods which requires much attention to select the best values for P I D gains.
For this project is used the Trial and Error Method.

Trial and Error Method is a simple method of PID controller tuning. We can tune the controller by first setting the
Ki and Kd values to zero and increase P term until system reaches the oscillating behavior. Once it is oscillating
Ki term needs to be adjusted to stop the oscilattion and finally D term is adjusted to get faster response.


