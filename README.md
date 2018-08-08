# Monitoring System

# Overview
This is a python and C++ library with ROS interface and designed to watch the System  during robot runing.

The computer system and the ROS communication will be watched .All message will be published in a ROS topic and show with a rqt-plugin.
User can write the fault detection and recovery itself by SDK.
More infomation please see the documentations and the usage


Features:

**monitoring_core:** other monitor use this pkg to add the value for topic /monitoring

**monitoring_visualization:** take the message form topic /monitoring and give it to GUI.User can see all the message in GUI window 

**monitoring_rqt:** plugin for rqt

**System monitor:** monitor for system,please see Documentation

**ROS monitor:** monitor for ROS,please see Documentation

**monitoring_fdir:** Fault detection and identification

**monitoring_bridge:** the bridge for Diagnostics

# Documentation

Documentation for monitor is here:

* [System monitor](https://ws02.rts.uni-hannover.de/monitoring/monitoring/tree/master/monitoring_monitors_system)

* [ROS monitor](https://ws02.rts.uni-hannover.de/monitoring/monitoring/tree/master/monitoring_monitors_ros)

* [monitoring_fdir](https://ws02.rts.uni-hannover.de/monitoring/monitoring/tree/master/monitoring_fdir)



# Installation

## Required:

Ros-kinetic (Ubuntu 16.04)

    sudo apt install libprocps4-dev
    sudo apt-get install python-ntplib


## building:

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

    cd catkin_ws/
    git clone https://ws02.rts.uni-hannover.de/monitoring/monitoring.git
    cd ..
    catkin_make


---

# Unit Tests and Ros test
run tests with:


    catkin_make run_tests_monitoring_monitors_ros
    catkin_make run_tests_monitoring_core



# Configuration

please see Documentation


---

# Usage

## Monitoring
The monitors all can be configure via a yaml-config files, you can find examples in the folder *config*.

You can write your own node. It is recommend to use the *include/ros_monitoring/monitors/monitormsg.h* interface

## Fault detection and identification
You can create your own FDI node with the SDK (*src/fdir/fdiSDK.cpp*). An example node is *src/fdir/fdiExample.cpp*

## Recovery
You can create your own Recovery node with the SDK (*src/recovery/recoverysdk.cpp*). An example implementation is *src/recovery/recoveryExample.cpp*

## GUI
There is a rqt plugin to visulaize the monitoring data. To use it, you need to start the *monitoring_visualization.launch*. This node aggregates all messages to send to a remote pc over the topic */monitoring/gui*. You can visualize the data with the *Monitor Viewer*

## Connect to Diagnostics
You can connect the monitoring systems to the diagnostics stack. There are two bridges. One for diagnostics to monitoring (node: *diagnostics_to_monitoring*) and one for monitoring to diagnostics (node: *monitoring_to_diagnostics*)

## Errorlevel
The Errorlevel is a value between 0.0 and 1.0 it can be any float value. They are further devided in 3 groups. Ok, Warn and Error states. Ok is between 0 and 0.35. Warn is till 0.7 and Error is till 1.0.
