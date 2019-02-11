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
import rospy
from monitoring_msgs.msg import *
from monitoring_core.monitor import Monitor
from monitoring_core.monitor import AggregationStrategies
from rosgraph_msgs.msg import TopicStatistics
from subprocess import check_output

ID = "TOPICSTATISTICS"
MONITOR_ = Monitor("multi_publish_monitor")
monitor_mode = 1

class Filter_type(object):
    DEFAULT = 0
    WHITELIST = 1
    BLACKLIST = 2

def init():
    """
    Init rospy node
    check for frequency parameter (default = 1Hz)
    check for filter_type (0 = default (list all), 1 = whitelist, 2 = black list)
    Return: frequency and filter_type, list_of_topic_names
    """
    topics = []
    ignored_topics = []
    if rospy.has_param('multi_publish_monitor/ignored_topics'):
        ignored_topics = rospy.get_param('multi_publish_monitor/ignored_topics')
    else:
        ignored_topics = ['/monitoring', '/tf', '/statistics']

    if rospy.has_param('multi_publish_monitor/monitor_mode'):
        monitor_mode = rospy.get_param('multi_publish_monitor/monitor_mode')
    else:
        monitor_mode = 1
    if rospy.has_param('multi_publish_monitor/frequency'):
        frequency = rospy.get_param('multi_publish_monitor/frequency')
    else:
        frequency = 1
    if rospy.has_param('multi_publish_monitor/filter_type'):
        filter_type = rospy.get_param('multi_publish_monitor/filter_type')
        if filter_type == "WHITELIST":
            filter_type = Filter_type.WHITELIST
        elif filter_type == "BLACKLIST":
            filter_type = Filter_type.BLACKLIST
        else:
            filter_type = Filter_type.DEFAULT
    else:
        filter_type = Filter_type.DEFAULT
    
    if filter_type != Filter_type.DEFAULT:
        try:
            topics = rospy.get_param('multi_publish_monitor/topics')
        except Exception as e:
            print e
    return frequency, filter_type, topics, ignored_topics

class MultiPubDetector():
    def __init__(self, frequency, filter_type, topics, ignored_topics, monitor):
        self.monitor = monitor
        self.freq = frequency
        self.filter_type = filter_type
        self.topics = topics
        self.ignored_topics = ignored_topics

        self.data = {}
        self.rtinfo = {}

        self.rostopic_info_timer = rospy.Timer(rospy.Duration(1.0 / frequency),
                                               self.timer_cb)
        self.statistics_sub = rospy.Subscriber("/statistics",
                                               TopicStatistics,
                                               self.statistics_cb)

    def timer_cb(self, timer):
        topics = check_output('rostopic list', shell=True).split('\n')
        for topic in topics:
            if not topic:
                continue
            if topic in self.ignored_topics:
                continue
            if self.filter_type == Filter_type.WHITELIST:
                if topic not in self.topics:
                    continue
            if self.filter_type == Filter_type.BLACKLIST:
                if topic in self.topics:
                    continue
            t = topic.replace("/","|")
            if t not in self.rtinfo.keys():
                self.rtinfo[t] = []
            temp = check_output("rostopic info "+topic,
                                shell=True)

            # ugly string parsing starts here
            temp = temp.split('\n')
            
            msg = temp[0].split(' ')[1]
            if 'rosgraph_msgs/Log' in msg:
                continue
            k, l = 0, 0
            for elem in temp:
                if 'Publishers: ' in elem:
                    k = temp.index(elem)
                if 'Subscribers: ' in elem:
                    l = temp.index(elem)
            pubs = temp[k:l]
            #print pubs
            subs = temp[l:]
            temp = []
            for elem in pubs:
                if not elem:
                    continue
                elif 'None' in elem:
                    continue
                elif 'Publishers' in elem:
                    continue
                # elem = ' * /gui_concat (http://ssd06:40143/)'
                temp.append(elem.split(' ')[2].replace("/","|"))
            # ugly string parsing ends here

            self.rtinfo[topic] = temp


    def statistics_cb(self, msg):
        name = msg.topic
        if name in self.ignored_topics:
            return
        if self.filter_type == Filter_type.WHITELIST:
            if name not in self.topics:
                return
        if self.filter_type == Filter_type.BLACKLIST:
            if name in self.topics:
                return

        name = name.replace("/","|")
        key_pub = msg.node_pub.replace("/","|")
        key_sub = msg.node_sub.replace("/","|")

        if name not in self.data:
            self.data[name] = {}
            self.data[name]["publisher"] = []
            self.data[name]["subscriber"] = []
        if not key_pub in self.data[name]["publisher"]:
            self.data[name]["publisher"].append(key_pub)
        if not key_sub in self.data[name]["subscriber"]:
            self.data[name]["subscriber"].append(key_sub)

    def get_infos_and_publish(self):
        for topic in self.data.keys():
            if len(self.data[topic]['publisher']) > 1:
                monitoring_string = "statistics_multipub/"+topic
                value = str(len(self.data[topic]['publisher']))
                unit = " "
                error = 0.0
                self.monitor.addValue(monitoring_string, value, unit,
                                      error, monitor_mode)
                temp = "statistics_multipub/"+topic+"/publisher"
                for i, pub in enumerate(self.data[topic]['publisher']):
                    value = pub
                    self.monitor.addValue(temp+str(i), value, unit,
                                      error, monitor_mode)
        for topic in self.rtinfo.keys():
            if len(self.rtinfo[topic]) > 1:
                monitoring_string = "registered_multipub/"+topic+"/num"

                value = str(len(self.rtinfo[topic]))
                unit = " "
                error = 0.0
                self.monitor.addValue(monitoring_string, value, unit,
                                      error, monitor_mode)
                temp = "registered_multipub/"+topic+"/publisher"

                for i, pub in enumerate(self.rtinfo[topic]):
                    value = pub
                    self.monitor.addValue(temp+str(i), value, unit,
                                      error, monitor_mode)


if __name__ == '__main__':
    rospy.init_node('multi_publish_monitor')

    freq, filter_t, topics, ignored_topics = init()
    RATE = rospy.Rate(freq)
    MONITOR_ = Monitor("multi_publish_monitor")
    mpdetect = MultiPubDetector(freq, filter_t, topics, ignored_topics, MONITOR_)

    while not rospy.is_shutdown():
        try:
            mpdetect.get_infos_and_publish()
        except rospy.ROSInterruptException:
            rospy.loginfo("ERROR")
        except Exception as e:
            print e
        finally:
            RATE.sleep()

