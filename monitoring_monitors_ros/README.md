# ROS monitor

# Overview
This is the monitor for ROS 


**ROS monitor:**

* **Topic-monitor**:this is a monitor for a topic,watch the daten in the topic 

* **TF-monitor**:this is a monitor for TF-tree ,check the TF-tree if it is in right configuation

* **Statistics-monitor**: this is a monitor for topic:Statistics,check the pub and sub if they  work right

* **node-monitor**: this is a monitor for ros-node,check the node if it works or not

* **node-resource-monitor** :TODO

# Configuration

## Topic monitor
You should configure one Topicmonitor per pc-system which only monitors local content to minimize networkload

## StatisticsMonitor
This monitor can only watch data flow from one node to another. If nobody subscribes the topic, the monitor is unable to monitor the content. If you record a rosbag, it subscribes the topic!

## node-monitor
you need to configure which node do you want to watch

## node-resource-monitor
you need to configure the blacklist and the whitelist for this monitor

## TF-monitor
you dont need to configure anything
