# Installation

Required:

sudo apt install libprocps4-dev



# Configuration

## Network monitor
if you want to check multiple network interfaces, start and configure one network-monitor-node per interface.



## Topic monitor
You should configure one Topicmonitor per pc-system which only monitors local content to minimize networkload

## StatisticsMonitor
This monitor can only watch data flow from one node to another. If nobody subscribes the topic, the monitor is unable to monitor the content. If you record a rosbag, it subscribes the topic!
