# ROS monitors

This package contains monitors to observe the ROS system. 

## Node monitor

The node monitor uses the rosping method to ping a list of ROS-Nodes and therefore estimate if they are still running and able to handle rospings. 

	frequency: 1					# Frequency used to ping the nodes
	nodes: [node_a, node_b]			# List of nodes to monitor

	monitoring:
	    frequency: 1				# Frecuency used to send monitored values
	monitor_mode : 2				# AggregationStrategies mode: 1 LAST, 2 FIRST, 3 MIN, 4 MAX, 5 AVG#


## Topic monitor

The topic monitor subscribes to all topics in a list and verifies that the are published on a certain frequency. It is based on the rostopic hz mechanism. For high bandwith topics this will cause a hiogh overhead since the topics are subscribed. The statistics monitor is a better solution in this case.

	frequency: 1									# Frequency used to poll rostopic hz
	monitoring:		
	    frequency: 2								# Frequency to send the monitored values
	topics: [{name: /monitoring, frequency: 2}]		# List of the monitored topics

## Node ressource monitor
## Map monitor
## Statistics monitor
## Tf monitor
## Topic value monitor