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
        self.host_name = socket.gethostname()
        self.node_name = rospy.get_name()
        self.name = self.host_name + self.node_name
        self.is_initialised = False
        self.autoPublishing = autoPublishing
        self.pub_count = 1
        self.reset_agg_dict = False
        self.publish_in_proc = False
        self.aggregation_dict = {}
        self.aggregation_dict[self.name] = {}
        self.switcher = {
                AggregationStrategies.LAST: 1,
                AggregationStrategies.FIRST: 2,
                AggregationStrategies.MIN: 3,
                AggregationStrategies.MAX: 4,
                AggregationStrategies.AVG: 5,
                }


    def init_ros(self, frequency=None):
        self.pub = rospy.Publisher('/monitoring', MonitoringArray, queue_size=1)

        if self.autoPublishing:
            try:
                frequency = rospy.get_param(rospy.get_name() + '/monitoring/frequency', 1)
                if not frequency:
                    rospy.logerr("frequency can not be 0, using 1")
                    frequency = 1
                duration = 1.0/frequency
                self.timer = rospy.Timer(rospy.Duration(duration), self.timercallback)
            except KeyError:
                rospy.logerr("monitoring frequency not set (%s/monitoring/frequency)", rospy.get_name())
                quit()
        rospy.loginfo("Monitoring frequency set to %f", frequency)
        self.is_initialised = True


    def timercallback(self, event):
        self.publish()


    def agg_to_int(self, agg):
        return self.switcher.get(agg, None)


    def int_to_agg(self, ival):
        for key in self.switcher.keys():
            if ival == self.switcher[key]:
                return key
        return None


    def addValue(self, key, value, unit, errorlevel, monitor_mode=AggregationStrategies.LAST):
        # preparation        
        if not self.is_initialised:
            self.init_ros()
        if self.reset_agg_dict:
            self.aggregation_dict[self.name] = {}
            self.reset_agg_dict = False
        # sanity checks
        if type(monitor_mode) == int:
            monitor_mode = self.int_to_agg(monitor_mode)
        elif type(monitor_mode) == float:
            monitor_mode = self.int_to_agg(int(monitor_mode))
        if " " in key:
            rospy.logwarn("[%s] whitespaces are not allowed in monitoring keys!", self.node_name)
        # data aggregation
        if key in self.aggregation_dict[self.name]:
            mode = self.aggregation_dict[self.name][key]['Mode']
            if mode in (5, 4, 3) and type(value) != float:
                if type(value) != int:
                    rospy.logwarn("With the current Aggregation Strategy for key: "+key+" your value has wrong type: "+str(type(value)))
                    rospy.logwarn("It has to be a numerical value in order to function. Doing nothing")
                    return    
            if mode == 5:
                self.aggregation_dict[self.name][key]['Num'] += 1
                self.aggregation_dict[self.name][key]['Sum'] += value
                self.aggregation_dict[self.name][key]['Error'] += errorlevel
            elif mode == 4:
                if value > self.aggregation_dict[self.name][key]['Value']:
                    self.aggregation_dict[self.name][key]['Value'] = value
                    self.aggregation_dict[self.name][key]['Error'] = errorlevel
            elif mode == 3:
                if value < self.aggregation_dict[self.name][key]['Value']:
                    self.aggregation_dict[self.name][key]['Value'] = value
                    self.aggregation_dict[self.name][key]['Error'] = errorlevel
            elif mode == 1:
                self.aggregation_dict[self.name][key]['Value'] = value
                self.aggregation_dict[self.name][key]['Error'] = errorlevel
        else:
            self.aggregation_dict[self.name][key] = {'Num' : 1 , 'Value' : value, 'Sum' : value, 'Mode' : self.agg_to_int(monitor_mode), 'Unit': str(unit), 'Error': errorlevel}


    def publish(self):
        self.publish_in_proc = True
        self.ma.header.stamp = rospy.Time.now()
        self.write_data()
        self.pub.publish(self.ma)
        self.resetMsg()
        self.pub_times = 1


    def write_data(self):
        mi = MonitoringInfo()
        mi.name = self.name
        mi.description = self.description
        mi.header.stamp = rospy.Time.now()
        self.ma.info.append(mi)
        for key in self.aggregation_dict[self.name].keys():
            kv = KeyValue()
            data = None
            try:
                data = self.aggregation_dict[self.name][key]
            except KeyError as e:
                #rospy.logwarn("{}".format(e))
                rospy.logwarn("Somehow key is missing: "+key)
                continue
            try:
                if data['Mode'] == 5:
                    kv.key = str(key)
                    kv.value = str(data['Sum']/(data['Num'] + 0.00001))
                    kv.unit = data['Unit']
                    kv.errorlevel = float(data['Error']/(data['Num']+0.0001))
                    self.ma.info[0].values.append(kv)
                elif data['Mode'] == 2:
                    kv.key = str(key)
                    kv.value = str(data['Value'])
                    kv.unit = data['Unit']
                    kv.errorlevel = data['Error']
                    self.ma.info[0].values.append(kv)
                elif data['Mode'] == 3:
                    kv.key = str(key)
                    kv.value = str(data['Value'])
                    kv.unit = data['Unit']
                    kv.errorlevel = data['Error']
                    self.ma.info[0].values.append(kv)
                elif data['Mode'] == 4:
                    kv.key = str(key)
                    kv.value = str(data['Value'])
                    kv.unit = data['Unit']
                    kv.errorlevel = data['Error']
                    self.ma.info[0].values.append(kv)
                elif data['Mode'] == 1:
                    kv.key = str(key)
                    kv.value = str(data['Value'])
                    kv.unit = data['Unit']
                    kv.errorlevel = data['Error']
                    self.ma.info[0].values.append(kv)
                else:
                    rospy.logerr("Key: "+str(key)+" has unknown Aggregation Strategy:" +str(data['Mode']))
            except Exception as e:
                pass


    def resetMsg(self):
        self.ma = MonitoringArray()
        self.reset_agg_dict = True


