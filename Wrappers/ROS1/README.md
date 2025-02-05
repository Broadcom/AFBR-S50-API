# Introduction #

This package contains ROS driver nodes for ToF Sensor AFBR-S50 with CAN and UART interface.

### System Configurations ###

* OS / ROS
	Ubuntu 16.04 / ROS Kinetic
* USB TO UART module
* AFBR-S50 mikroE sensor board (https://www.mikroe.com/bdc-afbr-s50-tof-sensor-board#/279-tof_sensor_board-bdc_afbr_s50mv85i)

# Quick Start

## Installation

### Installing ROS

Install "ROS Desktop Full" on Ubuntu PC.

- ROS Kinetic for Ubuntu 16.04
    - http://wiki.ros.org/kinetic/Installation/Ubuntu

## Installation of tof_driver ##


```
$ source /opt/ros/kinetic/setup.bash
$ mkdir -p ~/s50_tof_driver/src
$ cd s50_tof_driver/src/
$ catkin_init_workspace
$ git clone https://github.com/toffffffffff.git
$ cd ~/s50_tof_driver
$ catkin_make
$ source ~/s50_tof_driver/devel/setup.bash
```

### Connecting Tof sensors ###

* Connect a bunch tof sensors through the CAN interface (e.g. https://www.mikroe.com/bdc-afbr-s50-tof-sensor-board#/279-tof_sensor_board-bdc_afbr_s50mv85i) to form a daisy chain
* Connect the Tof sensor to the USB port of your Ubuntu PC via USB TO UART module

![connection](media/connection.png)


### Launching Software ###
#### Option 1 : Publish original data to ROS ####

* Open a new terminal and launch the raw data publisher.
```
$ source ~/s50_tof_driver/devel/setup.bash
$ chmod -R 777 ~/s50_tof_driver/
$ roslaunch raw_tof raw_tof_c.launch
```

#### Option 2 : PointCloud in Rviz ####

* Open a new terminal and launch the pointcloud2 publisher.
```
$ rosrun rviz rviz
$ source ~/s50_tof_driver/devel/setup.bash
$ chmod -R 777 ~/s50_tof_driver/
$ roslaunch pointcloud_tof pointcloud.launch
```
The image underneath shows an example turtlebot implementation of 5 x sensor boards (https://www.mikroe.com/bdc-afbr-s50-tof-sensor-board#/279-tof_sensor_board-bdc_afbr_s50mv85i)
showing a pointcloud of 5 x 32 pixels via Rviz.
![Rviz](media/Rviz.png)

Here is an application video showing the capability of AFBR-S50 sensor for cliff detection.
![cliff](media/cliff.mp4)