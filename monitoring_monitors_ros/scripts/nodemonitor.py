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
from rosnode import *
from monitoring_msgs.msg import *
from monitoring_core.monitor import Monitor
from monitoring_core.monitor import AggregationStrategies


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
	    
            if not rosnode_ping(node, 0.5):
                rospy.logwarn("Can not ping node: %s", node)
                monitor.addValue(node, "node unavailable", "", 0.5 , AggregationStrategies.LAST)
	    else:
		monitor.addValue(node, "node avialable", "", 0.0, AggregationStrategies.LAST)

        rate.sleep()


if __name__ == '__main__':
    try:
        nodemonitor()
    except rospy.ROSInterruptException:
        pass
