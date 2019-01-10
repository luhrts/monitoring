# System monitors

There are the monitor for System

## clock_difference-monitor

The clock_difference-monitor Checksum routine for Internet Protocol family headers. Measures the differences between machines' clocks using ICMP timestamp messages.

**Monitored Values:**

* None

**Warning/Errors:**

* TODO


**Parameters:**

* None


## CPU monitor

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

|     values      | unit  | Comment  | 
|-----------------|-------|----------|
|        RX       | Byte/s or Packet/s|     Speed of Receive       |
|        TX       | Byte/s or Packet/s|     Speed of Transmit      |
|      Load_RX    |  %    |       Speed of Receive in percentage   |
|      load_TX    |  %    |      Speed of Transmit in percentage   |


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

|     values      | unit  |            Comment           |
|-----------------|-------|------------------------------|
| signal_strength | dBm   |     WIFI strength in dBm     |
|     quality     |       |     WIFI quality             |
|      noise      |       |     WIFI noise               |


**Warning/Errors:**

None

**Parameters:**


	monitoring:
	  frequency: 1                     #Publish and calculation frequency
	monitor_mode : 1                   #AggregationStrategies mode: 1 LAST, 2 FIRST, 3 MIN, 4 MAX, 5 AVG#

## RAM-monitor

The RAM-monitor watch the RAM last
**Monitored Values:**

|     values      | unit  |                Comment            | 
|-----------------|-------|-----------------------------------|
|   total_used    |  kB   |    Usage of Ram                   |
| percentage_used |   %   |    Usage of Ram in percentage     |


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

|     values      | unit  |                                 Comment                         |
|-----------------|-------|-----------------------------------------------------------------|
|   ntp_offset    |   s   |  the time difference  between the client server and source      |
|   ntp_version   |       |                              version of ntp                     |
|     ntp_time    |       |                               Time in UTC                       |
|   ntp_leap      |       |                              leap seconds                       |
| ntp_time_unix   |       |                            Time in unix time                    |
| ntp_root_delay  |   s   |total roundtrip delay to the primary reference source at the root| 


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


* **ping-monitor**:  A pure python ping implementation using raw socket. Note that ICMP messages can only be sent from processes running as root. **Does not connect to GUI.**
use it by :



