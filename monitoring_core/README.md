# Monitoring Core

# Overview
Base Classes for other Monitors

**Description**
* **monitor.cpp/Class_Monitor**:Class Monitor - A class from which other monitors inherit.

* **init Function**:Monitor(ros::NodeHandle &n, std::string monitorDescription, bool autoPublishing)

* **alternative Init**:Monitor(std::string monitorDescription, bool autoPublishing)

* **description**:setup and publish a monitoring_msgs/MonitoringArray.h in a timer-Callback with a given frequency

* **include/monitoring_core/utils/monitoring_subscriber.h/Class_MonitoringSubscriber**
* **init Function**: MonitoringSubscriber (ros::NodeHandle& n)
 
* **description**: A class that subscribes to topic /monitoring and stores incomming messages in receivedArrays_

* **helper: waitAndSpin(ros::Duration dur = ros::Duration(0.1), int iterations = 50)**: Make sure the messages are received
* **helper: hasReceivedKey(std::string key)**: check if key - value is in the stored receivedArrays_
 
