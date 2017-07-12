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
[image4]: ./output/P01-D1.gif "Params = [0.1, 0, 1, ]"
[video4]: ./output/P01-D1.mov "Params = [0.1, 0, 1]"
[image5]: ./output/P01-D5.gif "Params = [0.1, 0, 5]"
[video5]: ./output/P01-D5.mov "Params = [0.1, 0, 5]"
[image6]: ./output/P01-D10.gif "Params = [0.1, 0, 10]"
[video6]: ./output/P01-D10.mov "Params = [0.1, 0, 10]"

[image7]: ./output/P0095-D1.35.gif "Params = [0.095, 0, 1.35]"
[video7]: ./output/P0095-D1.35.mov "Params = [0.095, 0, 1.35]"

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

or

1. Build and run: `./run.sh`


## PID Controller

* #### P component
...The Proportional term produces an output value proportional to the CTE value.
P term can be adjusted by multiplying the error with a constant value Kp called the proportional gain constant.
Using a small value for Kp parameter will result in a small change in the output (steering angle)
and using a high value will result in a larger output response. However if Kp parameter is too high the system
can start to oscillate around the target and can become unstable.

| P I D parameters | Result | Conclusion |
| --- | --- | --- |
| 5, 0, 0 | <img src="./output/P5.gif" width="320px" height="237px" /> | Definitely too high, vehicle is unstable.
| 1, 0, 0 | <img src="./output/P1.gif" width="320px" height="237px" /> | Still too high but a little bit better.
| 0.1, 0, 0 | <img src="./output/P01.gif" width="320px" height="237px" /> | Not bad, vehicle bounce from left to right but stays on track on a straight line. In order to have a faster response when turning D term should be used.

* #### I component
...The I term is proportional to both the magnitude of the error and the duration of the error.
The I term accelerates the movement of the process towards setpoint and eliminates the residual steady-state error that occurs with a pure P controller.

* #### D component
...The D term is proportional to the rate of change of CTE.
The D term improves settling time and stability of the system and reduces the overshooting and oscilattions caused by P term.

| P I D parameters | Result | Conclusion |
| --- | --- | --- |
| 0.1, 0, 10 | <img src="./output/P01-D10.gif" width="320px" height="237px" /> | Using both P and D parameters the vehicle is able to stay on the track. But with the current value of 10 for D params the movements are not so smooth as you will expect while driving a car.
| 0.1, 0, 5 | <img src="./output/P01-D5.gif" width="320px" height="237px" /> | Using a lower value for D term still keeps the vehicle on the track and turns now looks a little bit better.
| 0.1, 0, 1 | <img src="./output/P01-D1.gif" width="320px" height="237px" /> | Vehicle starts to turns quite smooth for an even lower D term value.

## Hyperparameters Tuning
...PID Controller must be tuned in order to suit with dynamics of the process to be controlled.
There are different type of tuning methods which requires much attention to select the best values for P I D gains.
For this project is used the Trial and Error Method.

Trial and Error Method is a simple method of PID controller tuning. We can tune the controller by first setting the
Ki and Kd values to zero and increase P term until system reaches the oscillating behavior. Once it is oscillating
Ki term needs to be adjusted to stop the oscilattion and finally D term is adjusted to get faster response.

Following the Trial and Error tuning method I first started only with P term using a high value, like 10 and I was expected to
see an oscillation of the vehicle around the target. The bouncing was too high and after checking with some lower values I finally decided that value of 0.1 gives quite good results (video results above).
Next using the I term was quite a challenge because I could not find a value for which to get some good results.
When I term value was high the vehicle went directly out of the track and for really small values the final output was not really effected. So I decided in the end to not use I term at all.
D term is used in order to improve system stability and to reduce the overshooting and oscilattions and that was obvious after setting this param because even when value was high the vehicle was able to stay on the track. Tuning for this parameter was necessary only for making the vehicle moves smoother.
After all this steps the final parameters were: P - 0.095, I - 0, D - 1.35 and the final output:

!

