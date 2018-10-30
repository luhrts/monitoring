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
 


## Errorlevel
The Errorlevel is a value between 0.0 and 1.0 it can be any float value. They are further devided in 3 groups. Ok, Warn and Error states. Ok is between 0 and 0.35. Warn is till 0.7 and Error is till 1.0.

## Aggregation Strategies
When monitoring a Value, not all values are required to be transmitted. Therefore an aggregation Startegiy can be selected:

- MIN 
- MAX 
- FIRST
- LAST
- AVG

You can change the mode in every config file.