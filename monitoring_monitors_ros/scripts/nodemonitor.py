#!/usr/bin/env python
import rospy
from rosnode import *
from monitoring_msgs.msg import *
from monitoring_core.monitor import Monitor


def nodemonitor():
    rospy.init_node("nodemonitor")
    try:
        frequency = rospy.get_param(rospy.get_name() + '/frequency')
        nodes = set(rospy.get_param(rospy.get_name() + '/nodes'))
    except KeyError:
        print "value not set"
        quit()

    rate = rospy.Rate(frequency)
    monitor = Monitor("nodemonitor")
    #pub = rospy.Publisher('/monitoring', MonitoringArray, queue_size=1)

    while not rospy.is_shutdown():  # main loop


        #currentNodes = set(get_node_names())
        #if(not nodes.issubset(currentNodes)):
        #    rospy.logwarn("missing nodes!")

        for node in nodes:
            if(not rosnode_ping(node, 1)):
                rospy.logwarn("Can not ping node: %s", node)
                monitor.addValue(node, "node unavailable", "", 0.5)


        rate.sleep()


if __name__ == '__main__':
    try:
        nodemonitor()
    except rospy.ROSInterruptException:
        pass
