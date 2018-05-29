#!/usr/bin/env python
import rospy
from monitoring_msgs.msg import *

class Monitor:
    def __init__(self, monitorDescription, autoPublishing = True):
        self.pub = rospy.Publisher('/monitoring', MonitoringArray, queue_size=1)
        self.ma = MonitoringArray()
        self.description = monitorDescription
        mi = MonitoringInfo()
        mi.name = rospy.get_name()
        mi.description = self.description
        self.ma.info.append(mi)

        if(autoPublishing):
            try:
                frequency = rospy.get_param(rospy.get_name() + '/monitoring/frequency')
                self.timer = rospy.Timer(rospy.Duration(1/frequency), self.publish)
            except KeyError:
                rospy.logerr("monitoring frequency not set (%s/monitoring/frequency)", rospy.get_name())
                quit()




    def addValue(self, key, value, unit, errorlevel):
        kv = KeyValue()
        kv.key = str(key)
        kv.value = str(value)
        kv.unit = str(unit)
        kv.errorlevel = errorlevel
        self.ma.info[0].values.append(kv)

    def publish(self):
        self.ma.header.stamp = rospy.Time.now()
        self.ma.info[0].header.stamp = rospy.Time.now()
        self.pub.publish(self.ma)
        self.resetMsg()

    def resetMsg(self):
        self.ma = MonitoringArray()
        mi = MonitoringInfo()
        mi.name = rospy.get_name()
        mi.description = self.description
        self.ma.info.append(mi)


