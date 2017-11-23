#!/usr/bin/env python
import rospy
from rosnode import *


def nodemonitor():
    rospy.init_node("nodemonitor")
    frequency = rospy.get_param(rospy.get_name() + '/frequency')
    nodes = set(rospy.get_param(rospy.get_name() + '/nodes'))
    rate = rospy.Rate(frequency)
    
    while not rospy.is_shutdown():  #main loop
        currentNodes = set(get_node_names())
        if(not nodes.issubset(currentNodes)):
            rospy.logwarn("missing nodes!")
        
        for node in nodes:
            if(not rosnode_ping(node, 1)):
                rospy.logwarn("Can not ping node: %s", node)
                
        rate.sleep()

if __name__ == '__main__':
    try:
        nodemonitor()
    except rospy.ROSInterruptException:
        pass