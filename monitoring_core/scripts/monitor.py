#!/usr/bin/env python
import rospy
from monitoring_msgs.msg import *

class monitor:
    def __init__(self, monitorDescription):
        self.pub = rospy.Publisher('/monitoring/error', MonitoringArray, queue_size=1)
        self.ma = MonitoringArray()
        self.description = monitorDescription
        mi = MonitoringInfo()
        mi.name = rospy.get_name()
        mi.description = self.description
        ma.info.append(mi)

    def addValue(self, key, value, unit, errorlevel):
        kv = KeyValue()
        kv.key = key
        kv.value = value
        kv.unit = unit
        kv.errorlevel = 0.5
        ma.info[0].values.append(kv)

    def publish(self):
        self.ma.header.stamp = rospy.Time.now()
        self.ma.info[0].header.stamp = rospy.Time.now()
        self.pub.publish(self.ma)
        self.resetMsg()

    def resetMsg(self):
        self.ma = MonitoringArray()
        mi = MonitoringInfo()
        mi.name = rospy.get_name()
        mi.description = monitorDescription
        ma.info.append(mi)


