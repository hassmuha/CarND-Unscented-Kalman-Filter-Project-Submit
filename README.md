# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric.

[//]: # (Image References)

[image1]: ./Result/sensor_fusion.png "Radar Laser SensorFusion"
[image2]: ./Result/Lidar_only.png "Laser only"
[image3]: ./Result/Radar_only.png "Radar_only"

---

* Source Code can be found [here](https://github.com/hassmuha/CarND-Unscented-Kalman-Filter-Project-Submit/tree/master/src)
* Compilation Result ./UnscentedKF can be found [here](https://github.com/hassmuha/CarND-Unscented-Kalman-Filter-Project-Submit/tree/master/build)

### NIS statistics and process noise selection
With the following process noise standard deviation values of acceleration and yaw acceleration:
std_a_ = 0.8;
std_yawdd_ = .3;

NIS statistics looks like below:
NIS above 7.815 for Radar = 4.01606%
NIS above 0.352 for Radar = 93.5743%
NIS above 5.991 for Lidar = 1.60643%
NIS above 0.103 for Lidar = 98.3936%

---

### Simulator snapshot for Sensor Fusion Example (Both Radar and Lidar Measurements)
![alt text][image1]

---

### Simulator snapshot for LIDAR only Measurements
![alt text][image2]

---

### Simulator snapshot for RADAR only Measurements
![alt text][image3]
