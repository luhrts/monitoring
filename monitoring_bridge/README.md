# Monitoring Bridge

# Overview
Interface between ROS-Diagnostics and Monitoring

**Description**
* **diagnosticstomonitoring.cpp/Class DiagnosticsToMonitoring**:A class that subscribes to /diagnostics topic and publishes on /monitoring

* **init Function**:DiagnosticsToMonitoring(ros::NodeHandle &node)

* **description**:sets up a Subscriber and Publisher, converts diagnostic_msgs to monitoring_msgs in diagnostics_callback, publishes the monitoring_msgs with the Rate of the Node.

* **monitoringtodiagnostics.cpp/Class_MonitoringToDiagnostics**: A class that subscribes to /monitoring topic and publishes on /diagnostics_agg
* **init Function**: MonitoringToDiagnostics(ros::NodeHandle& n)
 
* **description**: Same as DiagnosticsToMonitoring just reversed 
