#!/usr/bin/env python
'''
   Copyright (c) 2020, University of Hannover
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
from rospy.msg import AnyMsg
from monitoring_msgs.msg import *
from monitoring_core.monitor import Monitor
from monitoring_core.monitor import AggregationStrategies


class TopicObserver:
    def __init__(self, name, timeout):
        self.name = name
        self.timeout = timeout
        self.msg_missing = False
        self.oldness = 0.0
        self.last_recieved = rospy.Time.now()
        self._sub = rospy.Subscriber(name, AnyMsg, self.any_cb)
        # check the oldness with 2 times frequency of the timeout
        self._timer = rospy.Timer(rospy.Duration(timeout / 2.0), self.timer_cb)

    def any_cb(self, msg):
        last_recieved = rospy.Time.now()

    def timer_cb(self, tm):
        self.oldness = (rospy.Time.now() - last_recieved).to_sec()
        if (self.oldness > self.timeout):
            self.msg_missing = True
        else:
            self.msg_missing = False

def msgdetection():
    rospy.init_node("msgdetection")
    try:
        topics = rospy.get_param(rospy.get_name() + '/topics') #dict
        frequency = rospy.get_param(rospy.get_name() + '/frequency') # double
        timeout = rospy.get_param(rospy.get_name() + '/timeout') # double
    except KeyError:
        print "value not set"
        quit()

    rate = rospy.Rate(frequency)
    monitor = Monitor("msgdetection")
    topic_obs = []
    for topic in topics:
        topic_obs.append(TopicObserver(topic["name"],timeout))

    hz_monitors = {}
    for entry in topics:
        rthz = ROSTopicHz(-1, filter_expr=None)
        print entry['name']
        sub = rospy.Subscriber(entry['name'], rospy.AnyMsg, rthz.callback_hz)
        hz_monitors[entry['name']] = rthz

    while not rospy.is_shutdown():
        for tobs_obj in topic_obs:
            if tobs_obj.msg_missing:
                # no new msg recieved
                value = "Topic " + tobs_obj.name + " sends no data"
                monitor.addValue(tobs_obj.name, value, "", 0.8, AggregationStrategies.LAST)
                # TODO add tobs_obj.oldness if needed
            else:
                # everything okay
                monitor.addValue(tobs_obj.name, "okay", "", 0.0, AggregationStrategies.LAST)
        rate.sleep()


if __name__ == '__main__':
    try:
        msgdetection()
    except rospy.ROSInterruptException:
        pass
