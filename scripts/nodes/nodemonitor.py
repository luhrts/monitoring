#!/usr/bin/env python
import rospy
from rosnode import *
from ros_monitoring.msg import *
from help import fillMachineInfo


def nodemonitor():
    rospy.init_node("nodemonitor")
    frequency = rospy.get_param(rospy.get_name() + '/frequency')
    nodes = set(rospy.get_param(rospy.get_name() + '/nodes'))
    rate = rospy.Rate(frequency)
    
    pub = rospy.Publisher('/monitoring', MonitoringArray, queue_size=1)
    
    while not rospy.is_shutdown():  # main loop
        mi = MonitoringInfo()
        ma = MonitoringArray()
        ma.info.append(mi)
        mi.header.stamp = rospy.Time.now()
        mi.name = rospy.get_name()
        mi.description = "A Node-Monitor"
        fillMachineInfo(mi)
        
        currentNodes = set(get_node_names())
        if(not nodes.issubset(currentNodes)):
            rospy.logwarn("missing nodes!")
        
        for node in nodes:
            if(not rosnode_ping(node, 1)):
                rospy.logwarn("Can not ping node: %s", node)
                kv = KeyValue()
                kv.key = "node unavailable"
                str = node
                kv.value = str
                kv.errorlevel = 0.5
                mi.values.append(kv)
            
        if(not len(mi.values) == 0):  # stops publishing if there are no missing nodes
            pub.publish(ma)
        rate.sleep()


if __name__ == '__main__':
    try:
        nodemonitor()
    except rospy.ROSInterruptException:
        pass
