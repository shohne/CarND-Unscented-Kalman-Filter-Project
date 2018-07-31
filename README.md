## Extended Kalman Filter
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This repo contains the written code to complete the project **Extended Kalman Filter** on Udacity Self-Driving Car Nanodegree. The goal is to predict vehicle position and velocity from radar and lidar measures. 

Prerequisites
---
To run this project, it is necessary to have a **c++11** compiler and **cmake** (3.5 minimum version).

Installation
---
First, clone the repository:
```
git clone https://github.com/shohne/CarND-Extended-Kalman-Filter-Project.git
```
Change current directory:
```
cd CarND-Extended-Kalman-Filter-Project
```
Execute cmake to create the makefile:
```
cmake .
```
Create the executable file:
```
make
```
If everything was accordingly, there should be an executable file **ExtendedKF** on current directory.

Running the Application
---
First, start the executable:
```
./ExtendedKF
```
Now, the program shows in console:
```
Listening to port 4567
```
It is waiting for tcp/ip connection on port to 4567 to receive update measures from the car simulator.

Finally, run the simulator. An example for execution:

[ekf.m4v](ekf.m4v)

We could obtain a **root mean square error** as:

|Data|RMSE |
|----|-----|
|x   |0.108|
|y   |0.098|
|vx  |0.476|
|vy  |0.520|

Implementation Details
---
Please visit the [report.md](report.md) for more information about the algorithm pipeline.
