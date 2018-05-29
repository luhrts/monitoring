#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from monitoring_core.monitor import Monitor
from std_msgs.msg import String


def talker():
    rospy.init_node('simple_monitor', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    monitor = Monitor("simple_monitor_python")
    c = 0
    while not rospy.is_shutdown():
        monitor.addValue("count", c, "SIUnits", 0.0)
        c += 1
        rospy.loginfo("c: %i", c)
        monitor.publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
