#!/usr/bin/env python
'''
   Copyright (c) 2018, University of Hannover
                       Institute for Systems Engineering - RTS
                       Professor Bernardo Wagner
 
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:
 
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.
 
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
''' 
import socket
import rospy
from monitoring_msgs.msg import *
from enum import Enum

class AggregationStrategies(Enum):
	LAST = 1
	FIRST = 2
	MIN = 3
	MAX = 4
	AVG = 5
	
class Monitor(object):
    def __init__(self, monitorDescription, autoPublishing=True):
        self.ma = MonitoringArray()
        self.description = monitorDescription
        mi = MonitoringInfo()
        self.host_name = socket.gethostname()
        self.node_name = rospy.get_name()
        mi.name = self.host_name + self.node_name
        mi.description = self.description
        self.ma.info.append(mi)

        self.is_initialised = False
        self.autoPublishing = autoPublishing



       ##aggregation##
	self.pub_times = 0
	self.aggregation_dict = {}
	self.aggregation_dict[self.host_name + self.node_name] = {}

    def init_ros(self):
        self.pub = rospy.Publisher('/monitoring', MonitoringArray, queue_size=1)

        if self.autoPublishing:
            try:
                frequency = 1
                frequency = rospy.get_param(rospy.get_name() + '/monitoring/frequency', 1)
                if frequency == 0:
                    rospy.logerr("frequency can not be 0, using 1")
                    frequency = 1
                duration = 1.0/frequency
                self.timer = rospy.Timer(rospy.Duration(duration), self.timercallback)    #//DIVISION BY ZERO
            except KeyError:
                rospy.logerr("monitoring frequency not set (%s/monitoring/frequency)", rospy.get_name())
                quit()

        self.is_initialised = True

    def timercallback(self, event):
        self.publish()

    def addValue(self, key, value, unit, errorlevel, monitor_mode=AggregationStrategies.LAST):
	def aggregation(mode):
    	    switcher = {
        	1: AggregationStrategies.LAST,
        	2: AggregationStrategies.FIRST,
        	3: AggregationStrategies.MIN,
        	4: AggregationStrategies.MAX,
        	5: AggregationStrategies.AVG,
    	    }
	    return switcher.get(mode, None)
        if not self.is_initialised:
            self.init_ros()
        # Check if key contains whitespace
        if " " in key:
            rospy.logwarn("[%s] whitespaces are not allowed in monitoring keys!", self.node_name)
	kv = KeyValue()
	if (self.host_name + self.node_name) in self.aggregation_dict:
	    if key in self.aggregation_dict[self.host_name + self.node_name]:
	        if aggregation(monitor_mode) == AggregationStrategies.AVG:
		    if rospy.get_rostime() - self.aggregation_dict[self.host_name + self.node_name][key]['Duration'] < rospy.Duration(5):
	                self.aggregatin_dict[self.host_name + self.node_name][key]['num'] += 1
	                self.aggregation_dict[self.host_name + self.node_name][key]['Sum'] += value
		    else:
	                kv.key = str(key)
	                kv.value = str(self.aggregation_dict[self.host_name + self.node_name][key]['Sum']/(self.aggregation_dict[self.host_name + self.node_name][key]['num'] + 0.001))
	                kv.unit = str(unit)
	                kv.errorlevel = errorlevel
			self.aggregation_dict[self.host_name + self.node_name][key] = {'num' : 0 , 'Value' : 0, 'Sum' : 0, 'Duration' : rospy.get_rostime()}
	        elif aggregation(monitor_mode) == AggregationStrategies.FIRST and self.pub_times == 0:
	            kv.key = str(key)
	            kv.value = str(value)
	            kv.unit = str(unit)
	            kv.errorlevel = errorlevel
	            self.pub_times = 1
	        elif aggregation(monitor_mode) == AggregationStrategies.MAX:
	            if value > self.aggregation_dict[self.host_name + self.node_name][key]['Value']:
	                self.aggregation_dict[self.host_name + self.node_name][key]['Value']= value
	            kv.key = str(key)
	            kv.value = str(self.aggregation_dict[self.host_name + self.node_name][key]['Value'])
	            kv.unit = str(unit)
	            kv.errorlevel = errorlevel
	        elif aggregation(monitor_mode) == AggregationStrategies.MIN:
	            if value < self.aggregation_dict[self.host_name + self.node_name][key]['Value']:
	                self.aggregation_dict[self.host_name + self.node_name][key]['Value']= value
	            kv.key = str(key)
	            kv.value = str(self.aggregation_dict[self.host_name + self.node_name][key]['Value'])
	            kv.unit = str(unit)
	            kv.errorlevel = errorlevel
	        elif aggregation(monitor_mode) == AggregationStrategies.LAST:
	            kv.key = str(key)
	            kv.value = str(value)
	            kv.unit = str(unit)
	            kv.errorlevel = errorlevel
	        self.ma.info[0].values.append(kv)
	    else:
	        self.aggregation_dict[self.host_name + self.node_name][key] = {'num' : 0 , 'Value' : 0, 'Sum' : 0, 'Duration' : rospy.get_rostime()}
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
        mi.name = self.host_name + self.node_name
        mi.description = self.description
        self.ma.info.append(mi)
