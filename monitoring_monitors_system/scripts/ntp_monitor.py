#!/usr/bin/env python
# -*- coding: utf-8 -*-
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

from time import ctime
import rospy
from monitoring_core.monitor import Monitor
from monitoring_core.monitor import AggregationStrategies

import ntplib

import rosnode

if __name__ == '__main__':
    rospy.init_node('ntp_monitor', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    monitor = Monitor("ntp_monitor")

    ntp_servers = rospy.get_param('ntp_servers', rosnode.get_machines_by_nodes())

    offset_warn = rospy.get_param('abs_offset_warn', 1.0)
    offset_error = rospy.get_param('abs_offset_error', 5.0)

    ntp_client = ntplib.NTPClient()
    while not rospy.is_shutdown():
        for server in ntp_servers:
            try:
                response = ntp_client.request(server)
                error = 0.0
                if abs(response.offset) > offset_warn:
                    error = 0.6
                if abs(response.offset) > offset_error:
                    error = 1.0
                monitor.addValue(server+"/ntp_offset", response.offset, "s", error, 1)
                monitor.addValue(server+"/ntp_version", response.version, "", 0.0, 1)
                monitor.addValue(server+"/ntp_time", ctime(response.tx_time), "", 0.0, 1)
                monitor.addValue(server+"/ntp_time_unix", response.tx_time, "", 0.0, 1)
                monitor.addValue(server+"/ntp_leap", ntplib.leap_to_text(response.leap), "", 0.0, 1)
                monitor.addValue(server+"/ntp_root_delay", response.root_delay, "s", 0.0, 1)
            except (ntplib.NTPException, ntplib.socket.gaierror):
                monitor.addValue(server+"/ntp_error", "Server not reachable", "", 1.0, 1)
        rate.sleep()
