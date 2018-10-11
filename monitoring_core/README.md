# Monitoring Core

# Overview
There are the monitor for System

**Description**
* **monitor.cpp/Class_Monitor()**:Class Monitor - A class from which other monitors inherit.

* **init Function**:Monitor(ros::NodeHandle &n, std::string monitorDescription, bool autoPublishing)

* **alternative Init**:Monitor(std::string monitorDescription, bool autoPublishing)

* **description**:publish a monitoring_msgs/MonitoringArray.h in a timer-Callback with a given frequency
