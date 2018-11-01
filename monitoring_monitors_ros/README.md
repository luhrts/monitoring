# ROS monitors

This package contains monitors to observe the ROS system. 

## Node monitor

The node monitor uses the rosping method to ping a list of ROS-Nodes and therefore estimate if they are still running and able to handle rospings. 

**Monitored Values:**

* None

**Warning/Errors:**

* Error if Node is not available


**Parameters:**


	frequency: 1					# Frequency used to ping the nodes
	nodes: [node_a, node_b]			# List of nodes to monitor

	monitoring:
	    frequency: 1				# Frecuency used to send monitored values
	monitor_mode : 2				# AggregationStrategies mode: 1 LAST, 2 FIRST, 3 MIN, 4 MAX, 5 AVG#


## Topic monitor

The topic monitor subscribes to all topics in a list and verifies that the are published on a certain frequency. It is based on the rostopic hz mechanism. For high bandwith topics this will cause a hiogh overhead since the topics are subscribed. The statistics monitor is a better solution in this case.

**Monitored Values:**

* None

**Warning/Errors:**

* Error if Topic is not available


**Parameters:**


	frequency: 1									# Frequency used to poll rostopic hz
	monitoring:		
	    frequency: 2								# Frequency to send the monitored values
	topics: [{name: /monitoring, frequency: 2}]		# List of the monitored topics

## Node ressource monitor

The Node ressource monitor is able to monitor the ressources a node uses. Using the psutil python library the monitor is able to monitor a number of values.

**Monitored Values:**


|   |   |   |   |   |
|---|---|---|---|---|
|   |   |   |   |   |
|   |   |   |   |   |
|   |   |   |   |   |

* children
* cmdline
* connections
* cpu_afinity
* cpu_percent
* create_time
* cwd
* exe
* gids
* ionice
* is_running
* memory_info_rss
* memory_info_vms
* memory_info_ex
* memory_maps
* memory_percent
* name
* nice
* num_ctx_switches_voluntary
* num_ctx_switches_involuntary
* num_fds
* num_handles
* num_threads
* open_files
* parent
* pid
* ppid
* rlimit
* status
* terminal
* threads
* uids
* username

**Warning/Errors:**

* None so far


**Parameters:**


	frequency: 1
	# 0 = default(list all), 1 = whitelist, 2 = blacklist
	filter_type: 0

	monitor_mode: 3
	#AggregationStrategies mode: 1 LAST, 2 FIRST, 3 MIN, 4 MAX, 5 AVG#

## Map monitor
## Statistics monitor
## Tf monitor
## Topic value monitor