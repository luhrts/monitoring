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
	nodes: [node_a, node_b]			        # List of nodes to monitor

	monitoring:
	    frequency: 1				# Frecuency used to send monitored values
	monitor_mode : 2				# AggregationStrategies mode: 1 LAST, 2 FIRST, 3 MIN, 4 MAX, 5 AVG#


## Topic monitor

The topic monitor subscribes to all topics in a list and checks whether they are published with a certain frequency. It is based on the rostopic hz mechanism. For high bandwith topics this leads to a high overhead, as the topics are subscribed. The statistics monitor is a better solution in this case.

**Monitored Values:**

* None

**Warning/Errors:**

* Error if Topic is not available


**Parameters:**


	frequency: 1									# Frequency used to poll rostopic hz
	monitoring:		
	    frequency: 2								# Frequency to send the monitored values
	topics: [{name: /monitoring, frequency: 2}]		                        # List of the monitored topics

## Node ressource monitor

The Node ressource monitor is able to monitor the ressources used by a node. Using the psutil python library the monitor can monitor a number of values.

**Monitored Values:**


|     values      | unit  |          Comment                   |
|-----------------|-------|------------------------------------|
|     children    |       |                                    |
|     cmdline     |       |                                    |
|   connections   |       |                                    |
|   cpu_afinity   |       |       Afinity of CPU               |
|   cpu_percent   |   %   |      Usage of CPU  in percentage   |
|   create_time   |  sec  |Create time for node in Linux time  |
|       cwd       |       |      Current working directory     |
|       exe       |       |                                    |
|       gids      |       |          Group Identifier          |
|      ionice     |       |  I/O scheduling class and priority |
|    io_counters  | byte  |       I/O accounting information   |
|    is_running   |       |                                    |
| memory_info_rss | byte  |   Resident Set Size of memory      |
| memory_info_vms | byte  |    Size of  Virtual Memory         |
| memory_info_ex  |       |                                    |
|   memory_maps   |       |                                    |
| memory_percent  |   %   |    Memory usage in percentage      |
|      name       |       |            Name of node            |
|      nice       |       |                                    |
|num_ctx_switches_voluntary  |      |     Voluntary context switch      |
|num_ctx_switches_involuntary|      |    Involuntary context switches   |
|     num_fds     |       |                                    |
|   num_handles   |       |                                    |
|   num_threads   |       |                                    |
|    open_files   |       |                                    |
|      parent     |       |          Parent of node            |
|       pid       |       |       Prozess ID of node           |
|       ppid      |       |       Parent Process Id of node    |
|      rlimit     |       |          Resource limits           |
|      status     |       |                                    |
|     terminal    |       |                                    |
|     threads     |       |          Threads of node           |
|      uids       |       |             User ID                |
|     username    |       |         Username of node           |


**Warning/Errors:**

* None so far


**Parameters:**


	frequency: 1                              # Frequency used to poll rostopic hz
	filter_type: 0                            # 0 = default(list all), 1 = whitelist, 2 = blacklist
	monitor_mode: 3                           #AggregationStrategies mode: 1 LAST, 2 FIRST, 3 MIN, 4 MAX, 5 AVG#

* see config/statisticsconfig.yaml

## Map monitor

**TODO**

## TF monitor

This is a monitor for TF-tree in ROS, check the TF-tree if it is in right configuration.

**Monitored Values:**

|     values      | unit  | Comment                |
|-----------------|-------|------------------------|
|        dx       |  m    |Variance in x direction |
|        dy       |  m    |Variance in y direction |
|        dz       |  m    |Variance in z direction |
|       drow      |degree |Variance of Rotation around x direction|
|      dpitch     |degree |Variance of Rotation around y direction|
|       dyaw      |degree |Variance of Rotation around z direction|


**Warning/Errors:**

* Error if static frame, parent and autohrity of the frame Changed
* Error if the loop and Seperation in the tf tree

**Parameters**

* **TODO**

## Statistics Monitor

The Statistics Monitor monitors advanced information about topics. It uses topic-statistics, so it must be enabled to  ros-system-start.

**Monitored Values:**

* **TODO**

**Warning/Errors:**

* **TODO**

**Parameters:**

	monitoring:
	  frequency: 1                                  # Frequency to send the monitored values
	monitor_mode : 5                                # AggregationStrategies mode: 1 LAST, 2 FIRST, 3 MIN, 4 MAX, 5 AVG#
	timeTilDelete: 3                                # Time for delete value from the  Message
	topics: [groupe_1,groupe_2]                     # List of titel name for the following groupe 
	
	groupe_1:
	  topic: /topic_1                               # The 'Topic' value in statistic topic,Which user want to watch by statistic monitor
	  source: /Publisher                            # Publisher for the 'Topic'
	  destination: /Subscriber                      # Subscriber for the 'Topic'
	  frequency: 1                                  # Publish frequency of the 'Topic' that User have configured
	  dFrequency: 0.5                               # Tolerance of the frequency
	  size: 0                                       # Size of the message in the 'Topic'
	  dSize: 1                                      # Tolerance of the size
	
	  type: geometry_msgs::Twist                    # Message tpye of the 'Topic'
	  errorlevel: 0.5                               # errorlevel of the Warn message


## Topic value monitor

The Topic value monitor is used to watch a message's' value that is published on a topic.
**Currently under development**

**Monitored Values:**

* **TODO**

**Warning/Errors:**

* **TODO**


**Parameters:**

* **TODO**
	
	
