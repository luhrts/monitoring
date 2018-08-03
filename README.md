# Monitoring System

# Overview
This is a python and C++ library with ROS interface .It designed to watch the System  during robot runing.

The following Features is contained in this Lib:

**monitoring_core**

**monitoring_visualization**

**monitoring_rqt**

**System monitor:**

* clock_diference-monitor

* cpu-monitor

* network-monitor

* wifistrength-monitor

* Ram-monitor

* ntp-monitor

* ping-monitor

**ROS monitor:**

* Topic-monitor

* TF-monitor

* Statistics-monitor

* node-monitor

* node-resource-monitor


# Documentation

Documentation for monitor is here:

* [System monitor](https://ws02.rts.uni-hannover.de/monitoring/monitoring/tree/master/monitoring_monitors_system)

* [ROS monitor](https://ws02.rts.uni-hannover.de/monitoring/monitoring/tree/master/monitoring_monitors_system)



# Installation

Required:

Ros-kinetic (Ubuntu 16.04)

sudo apt install libprocps4-dev
sudo pip install ntplib

---

# Unit Tests and Ros test
run tests with:


catkin_make run_tests_monitoring_monitors_ros

catkin_make run_tests_monitoring_core



# Configuration

## Network monitor
if you want to check multiple network interfaces, start and configure one network-monitor-node per interface.


## Topic monitor
You should configure one Topicmonitor per pc-system which only monitors local content to minimize networkload

## StatisticsMonitor
This monitor can only watch data flow from one node to another. If nobody subscribes the topic, the monitor is unable to monitor the content. If you record a rosbag, it subscribes the topic!

## cpu-monitor
you should configure the work frequenz and what kind of things you want to monitore for CPU
 
## wifistrength-monitor
you need to configure the work frequenz and WIFI-Interface

## Ram-monitor
you need to configure the work frequenz and unit type
 
## node-monitor
you need to configure which node do you want to watch

## node-resource-monitor
you need to configure the blacklist and the whitelist for this monitor

## ping-monitor ntp-monitor TF-monitor clock_diference-monitor 
 you dont need to configure anything


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
There is a rqt plugin to visulaize the monitoring data. To use it, you need to start the *gui_msg_concat_node* node. This node aggregates all messages to send to a remote pc over the topic */monitoring/gui*. You can visualize the data with the *Monitor Viewer*

## Connect to Diagnostics
You can connect the monitoring systems to the diagnostics stack. There are two bridges. One for diagnostics to monitoring (node: *diagnostics_to_monitoring*) and one for monitoring to diagnostics (node: *monitoring_to_diagnostics*)

## Errorlevel
The Errorlevel is a value between 0.0 and 1.0 it can be any float value. They are further devided in 3 groups. Ok, Warn and Error states. Ok is between 0 and 0.35. Warn is till 0.7 and Error is till 1.0.
