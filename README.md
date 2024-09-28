# DDoS MSF Localizer

## Introduction
The project aims to implement the content of Lin's thesis and is divided into three main parts: IMU Calibration, Multi-Sensor Fusion (MSF) Localizer, and Data Collection.

* IMU Calibration:
    * Collects 3 minutes of static IMU calibration data, averages the results, and obtains the final static calibration values.
    * If the testing location changes, recalibration is required.

* MSF Localizer:
    * It integrates multiple sensors (IMU, odometry, LiDAR, GNSS) for a localization solution, with a focus on defending against Distributed Denial of Service (DDoS) attacks.

* Data Collection:
    * During the localization process, executing the command will collect various positioning data to facilitate subsequent analysis for the thesis.

---
## How to Install 
* cd ~/catkin_ws && catkin_make --only-pkg-with-deps ddos_msf_localizer

---
## How to Use
### imu_calibration
* The vehicle must remain stationary. After running the command, static IMU data will be collected for three minutes. 
    * rosrun ddos_defense_system imu_calibration
* The final result should be entered into the corresponding section of the ddos_msf_localizer launch file.

### ddos_msf_localizer
* After completing the calibration, the next step is to execute this command.
    * roslaunch ddos_defense_system ddos_msf_localizer.launch 

### data_collect
* Executing the command will collect various positioning data.
    * roslaunch ddos_defense_system data_collect.launch 