# ROS Monitoring

The ROS Monitoring Tools helps to analyse and improve ROS Systems. It collects information about the coputer, operating system and ros environment and filters them. All Values and detected Warnings and Errors can be visualised in a RQT Plugin.

# Overview

This tool is designed to collect and observe system values, detect and identify failures and to perform recovery when possible. It can handle a number of distributed Monitors. Some gernal Monitors are included within this package, for all other nodes libraries for C++ and Python exist to integrate the Monitoring into custom Nodes. 

All Monitoring-Data is aggregated and can be visualised by two different RQT-Plugins (a varriant of the ROS Monitor and the ROS Plot, modified to support Monitoring)

Besides visualisation, Faultdetection and Recovery are supported. 


![Overview_Image_Monitoring](monitoring/images/Monitoring_Overview.png)


# Installation

The monitoring tools are tested on:

- Kinetik
- Melodic

## Requirements:
To use all features the following packages are required.

    sudo apt install libprocps4-dev python-ntplib

## Building from source:

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using catkin_make.

# Packages

* [monitoring_bag_plot](monitoring_bag_plot) contains tools to plot from rosbags, usefull for offline analysis

* [monitoring_monitors_system](monitoring_monitors_system) contains montiros to observer certain non ros-specific values of the system, currently the following monitors are included:

	* clockdifff_monitor
	* cpu_monitor
	* cpu frequency monitor
	* network interface monitor
	* ping monitor
	* ram monitor
	* wifi monitor

* [monitoring_monitors_ros](monitoring_monitors_ros)

* [monitoring_bridge](monitoring_visualization)

* [monitoring_fdir](monitoring_fdir)


# Usage

## Example




## GUI
There is a rqt plugin to visulaize the monitoring data. To use it, you need to start the *monitoring_visualization.launch*. This node aggregates all messages to send to a remote pc over the topic */monitoring/gui*. You can visualize the data with the *Monitor Viewer*

## Connect to Diagnostics
You can connect the monitoring systems to the diagnostics stack. There are two bridges. One for diagnostics to monitoring (node: *diagnostics_to_monitoring*) and one for monitoring to diagnostics (node: *monitoring_to_diagnostics*)


