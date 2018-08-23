# System monitorui

# Overview
There are the monitor for System

**Description**
* **clock_diference-monitor**: Checksum routine for Internet Protocol family headers.This routine is very heavily used in the network,code should be modified for each CPU to be as fast as possible.This implementation is TAHOE version.

* **cpu-monitor**:watch the CPU frequence ,last and temperatur

* **network-monitor**:watch the network,speed and the number of the package will be watched and network error will be checked

* **wifistrength-monitor**: watch the wifi Strength 

* **Ram-monitor**:watch the RAM last

* **ntp-monitor**:checkout the time by use NTP

* **ping-monitor**:  A pure python ping implementation using raw socket. Note that ICMP messages can only be sent from processes running as root.**Does not connect to GUI.** use it by :

    sudo python $monitoring_path/monitoring_monitors_system/scripts/ping_monitor.py
# Configuration

## Network monitor
if you want to check multiple network interfaces, start and configure one network-monitor-node per interface.

## cpu-monitor
you should configure  what kind of things you want to monitore for CPU
 
## wifistrength-monitor
you need to configure the  WIFI-Interface

## Ram-monitor
you need to configure the unit type

## ping-monitor
need to configure the IP-adresse in scripts/ping_monitor.py

## ntp-monitor
dont need to configure anything

## clock_diference-monitor 
TODO

