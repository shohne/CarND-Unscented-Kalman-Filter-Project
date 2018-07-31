# **Extended Kalman Filter**

The goals / steps of this project are the following:
* Predict position and velocity to a vehicle;
* Receive radar and lidar information from vehicle and update its state using a [Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter).

### Inplementation Details

There are 3 main steps in this code:
1. Initialization;
2. Prediction;
3. Update state;

All these 3 steps are in the file **src/FusionEKF.cpp**, the *initialization* is on class *FusionEKF* constructor and beginning of FusionEKF::ProcessMeasurement method. The *Prediction** and *Update* are in **FusionEKF::ProcessMeasurement**.

There is a auxiliary class *Tools* that computes:
1. RMSE [root mean square error](https://en.wikipedia.org/wiki/Root-mean-square_deviation), method **Tools::CalculateRMSE**;
2. Jacobian Matrix for Radar, method **Tools::CalculateJacobian**;
3. Tools::AdjustAngle that, from a given angle, returns the correspondent between -&#960; and &#960;.

