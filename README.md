# Two-wheeled Self-Balancing Robot

My attempt at building a two-wheeled self-balancing robot. The project started as a personal one and
later evolved as a University project. I'll upload the university report to this repo as soon as it is complete. This repo will also be
supported by a blog post on [emamaker.com](https://emamaker.com)

The physical model of the robot has been derived by calculatin the Lagrangian function. This model
is heavily non-linear and has been linearized around the 0° point (equilibrium).

The linearized model has been reconstructed in MatLab Simulink and a discrete-time PID controller
has been calculated by trial and error.

The robot has later been built using a Raspberry Pi Pico microcontroller paired with a MPU6050
6-axis IMU, from which the pitch angle is derived with the use of a [Madgwick](https://ahrs.readthedocs.io/en/latest/filters/madgwick.html) filter.
(Initially this was a simple complementary filter, but it was way to noise, check the blog post).
The two motors (Hitec HS-322HD servomotors, check blog post for the choice
     of motors) are driven by a L298N motor driver. Everything is powered by a 2S 7.4V LiPo battery.

The microcontroller firmware has been implemented in Arduino-C with the use of the ArduPID library
 for the PID Controller and FastIMU for reading the IMU. A reimplementation using the RPi Pico own
 SDK will probably follow. This repo includes both the old firmware using a complementary filter and
 the new one using the Madgwick filter, the latter is the one actually used on the robot.

The structure (as in chassis) of the robot has been designed in OnShape and realized using laser-cut
wood and 3d-printed parts, check the blog post for details about the structure.
