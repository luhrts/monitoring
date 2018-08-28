#!/usr/bin/env python
import socket
import rospy
from monitoring_msgs.msg import *



class Monitor(object):
    def __init__(self, monitorDescription, autoPublishing = True):
        self.ma = MonitoringArray()
        self.description = monitorDescription
        mi = MonitoringInfo()
        self.host_name = socket.gethostname()
        self.node_name = rospy.get_name()
        mi.name =  self.host_name + self.node_name
        mi.description = self.description
        self.ma.info.append(mi)

        self.is_initialised = False
        self.autoPublishing = autoPublishing

    def init_ros(self):
        self.pub = rospy.Publisher('/monitoring', MonitoringArray, queue_size=1)

        if self.autoPublishing:
            try:
                frequency = 1
                frequency = rospy.get_param(rospy.get_name() + '/monitoring/frequency', 1)
                if(frequency == 0):
                    rospy.logerr("frequency can not be 0, using 1")
                    frequency = 1
                duration =1.0/frequency
                self.timer = rospy.Timer(rospy.Duration(duration), self.timercallback)       #//DIVISION BY ZERO
            except KeyError:
                rospy.logerr("monitoring frequency not set (%s/monitoring/frequency)", rospy.get_name())
                quit()

        self.is_initialised = True

    def timercallback(self, event):
        self.publish()

    def addValue(self, key, value, unit, errorlevel):
        if not self.is_initialised:
            self.init_ros()

        # Check if key contains whitespace
        if " " in key:
            rospy.logwarn("[%s] whitespaces are not allowed in monitoring keys!", self.node_name)

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
        mi.name =  self.host_name + self.node_name
        mi.description = self.description
        self.ma.info.append(mi)
