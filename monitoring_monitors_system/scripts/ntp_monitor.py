#!/usr/bin/env python
# -*- coding: utf-8 -*-
from time import ctime
import rospy
from monitoring_core.monitor import Monitor

import ntplib


if __name__ == '__main__':
    rospy.init_node('ntp_monitor', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    monitor = Monitor("ntp_monitor")

    ntp_servers = rospy.get_param('ntp_servers', ['pool.ntp.org', '130.0.0.0'])
    ntp_client = ntplib.NTPClient()
    while not rospy.is_shutdown():
        for server in ntp_servers:
            try:
                response = ntp_client.request(server)
                monitor.addValue(server+"/ntp_offset", response.offset, "s", 0.0, 1)
                monitor.addValue(server+"/ntp_version", response.version, "", 0.0, 1)
                monitor.addValue(server+"/ntp_time", ctime(response.tx_time), "", 0.0, 1)
                monitor.addValue(server+"/ntp_time_unix", response.tx_time, "", 0.0, 1)
                monitor.addValue(server+"/ntp_leap", ntplib.leap_to_text(response.leap), "", 0.0, 1)
                monitor.addValue(server+"/ntp_root_delay", response.root_delay, "s", 0.0, 1)
            except (ntplib.NTPException, ntplib.socket.gaierror):
                monitor.addValue(server+"/ntp_error", "Server not reachable", "", 1.0, 1)

        rate.sleep()
