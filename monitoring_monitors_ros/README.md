# ROS monitor

# Overview
This is the monitor for ROS 


**ROS monitor:**

* **Topic-monitor**:this is a monitor for a topic,watch the daten in the topic .when you just want to watch a specific topic,use this monitore.

* **TF-monitor**:this is a monitor for TF-tree ,check the TF-tree if it is in right configuation.User can see the transform for each frame.Parent Change ,Autohrity Change ,Speration tree,loop tree  for tf_tree will be checked and warned.

* **Statistics-monitor**: this is a monitor for topic:Statistics. the publisher und subscriber for all the topic will be checked if it work in right frequency and size.

* **node-monitor**: this is a monitor for ros-node,check the node if it works or not 

* **node-resource-monitor** :  lists all running rosnodes and displays all values the psutil package is providing for each node

# Configuration

## Topic monitor
You should configure one Topicmonitor per pc-system which only monitors local content to minimize networkload

## StatisticsMonitor
This monitor can only watch data flow from one node to another. If nobody subscribes the topic, the monitor is unable to monitor the content. If you record a rosbag, it subscribes the topic!

## node-monitor
you need to configure which node do you want to watch

## node-resource-monitor
you need to chose which mode do you want to use,blacklist or whitelist.configure it im start_ros_monitorings.launch.Then you have to configure filter for each node im node_ressource_monitor_filter.yaml.

## TF-monitor
you dont need to configure anything
