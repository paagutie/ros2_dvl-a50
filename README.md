# ros2_dvl-a50
ROS2 package to read the Water Linked DVL-A50 Sensor (Doppler Velocity Log) using a TCP/IP Socket. 

> Note: **This repository is deprecated!** Please visit its new version at [dvl-a50](https://github.com/paagutie/dvl-a50).

### Description
This package contains a python script (dvl-A50.py) with which it's possible to publish information read from the sensor (in json format) over a TCP/IP socket. The /dvl/data topic is built using the custom message structure contained in the "rov_msgs" package.

### Requirements
- ROS2 Foxy
- Ubuntu 20.04

## Installation
```
$ git clone https://github.com/paagutie/ros2_dvl-a50.git
$ cd ros2_dvl-a50
$ source /opt/ros/foxy/setup.bash
$ colcon build
```
### Usage
```
$ source install/local_setup.bash
$ ros2 run rov_io dvl-A50.py
```

## Licence
