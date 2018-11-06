# System monitors

There are the monitor for System

## clock_diference-monitor

The clock_diference-monitor Checksum routine for Internet Protocol family headers.

**Monitored Values:**

* None

**Warning/Errors:**

* TODO


**Parameters:**

* None


## Topic monitor

The topic monitor subscribes to all topics in a list and verifies that the are published on a certain frequency. It is based on the rostopic hz mechanism. For high bandwith topics this will cause a hiogh overhead since the topics are subscribed. The statistics monitor is a better solution in this case.

**Monitored Values:**

* None

**Warning/Errors:**

* Warn if temperature and load are over warn value
* Error if temperature and load are over error value


**Parameters:**


	avarage: true                             #the Average of the Last Minute. New info every 5s (Calculated by Kernel)
	percent: true                             #The Average over frequency/1s in percent
	percentPerCore: true                      #like percent but for every core
	temperature: true                         #temperature of the CPU
	temperature_warn: 80                      #warn value for temperatur (default:80)
	temperature_error: 85                     #error value for temperatur (default:95)
	percent_total_warn: 80                    #warn value for temoeratur (default:80)
	percent_total_error: 85                   #error value for temperatur (default:95)
	
	monitoring:
	  frequency: 1                            #Publish and calculation frequency
	
	monitor_mode : 1                          #AggregationStrategies mode: 1 LAST, 2 FIRST, 3 MIN, 4 MAX, 5 AVG#


## Network-monitor

The network monitor watch the network,speed and the number of the package and network error will be checked
**Monitored Values:**

|     values      | unit  | Comment  |   |   |
|-----------------|-------|----------|---|---|
|        RX       | Byte/s or Packet/s|          |   |   |
|        TX       | Byte/s or Packet/s|          |   |   |
|      Load_RX    |  %    |          |   |   |
|      load_TX    |  %    |          |   |   |

* RX
* TX
* Load_RX
* load_TX


**Warning/Errors:**

The following Error will be demonstrated:

* RX_CRC_Errors
* RX_Dropped
* RX_Errors
* RX_FIFO_Errors
* RX_Frame_Errors
* RX_Length_Errors
* RX_Missed_Errors
* RX_Over_Errors
* TX_Carrier_Errors
* TX_Dropped
* TX_Errors
* TX_FIFO_Errors
* TX_Heartbeat_Errors
* TX_Window_Errors
* TX_Aborted_Errors


**Parameters:**


	monitoring:
	  frequency: 1                     #Publish and calculation frequency

	monitor_mode : 1                   #AggregationStrategies mode: 1 LAST, 2 FIRST, 3 MIN, 4 MAX, 5 AVG#
	
	bytes: true                        #networktrafic in bytes
	packets: true                      #networktrafic in packets
	load: true                         #networkload scala 0..1
	networkerrors: true
	networkthroughput: 100             #maximal possible throughput in MBit/s.
	networkinterface: enp0s31f6        #the networkinterface to monitor. If you need to monitor more interfaces, start multiple nodes with diffrent configurations

## wifi-monitor

The wifi monitor watch the wifi Strength
**Monitored Values:**

|     values      | unit  | Comment  |   |   |
|-----------------|-------|----------|---|---|
| signal_strength | dBm   |          |   |   |
|     quality     |       |          |   |   |
|      noise      |       |          |   |   |

* signal_strength
* quality
* noise

**Warning/Errors:**

None

**Parameters:**


	monitoring:
	  frequency: 1                     #Publish and calculation frequency
	monitor_mode : 1                   #AggregationStrategies mode: 1 LAST, 2 FIRST, 3 MIN, 4 MAX, 5 AVG#

## RAM-monitor

The RAM-monitor watch the RAM last
**Monitored Values:**

|     values      | unit  | Comment  |   |   |
|-----------------|-------|----------|---|---|
|   total_used    |  kB   |          |   |   |
| percentage_used |   %   |          |   |   |

* total_used
* percentage_used

**Warning/Errors:**

None

**Parameters:**


	monitoring:
	  frequency: 1                         #Publish and calculation frequency
	percent: true                          #RAM load in %
	used: true                             #RAM load in KB

	monitor_mode : 2                       #AggregationStrategies mode: 1 LAST, 2 FIRST, 3 MIN, 4 MAX, 5 AVG#


## ntp-monitor

The ntp-monitor checkout the time by use NTP
**Monitored Values:**

|     values      | unit  | Comment  |   |   |
|-----------------|-------|----------|---|---|
|   ntp_offset    |   s   |          |   |   |
|   ntp_version   |       |          |   |   |
|   total_used    |       |          |   |   |
|     ntp_time    |       |          |   |   |
|   ntp_leap      |       |          |   |   |
| ntp_time_unix   |       |          |   |   |
| ntp_root_delay  |   s   |          |   |   |

* ntp_offset
* ntp_version
* ntp_time
* ntp_time_unix
* ntp_leap
* ntp_root_delay



**Warning/Errors:**

Error if connect ntp failed

**Parameters:**

None

## Ping-monitor

The ping-monitorr is a pure python ping implementation using raw socket
**Monitored Values:**

None

**Warning/Errors:**

None

**Parameters:**

	machines: ["130.75.137.10", "google.de"]

	monitoring:
	  frequency: 1   #Publish and calculation frequency


* **ping-monitor**:  A pure python ping implementation using raw socket. Note that ICMP messages can only be sent from processes running as root.**Does not connect to GUI.** use it by :



