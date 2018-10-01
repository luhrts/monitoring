#!/usr/bin/env python
import time
from time import sleep

import rospy
from rostopic import *
from monitoring_msgs.msg import *
from monitoring_core.monitor import Monitor
from monitoring_core.monitor import AggregationStrategies



def topicmonitor():
    rospy.init_node("topicmonitor")
    try:
        topics = rospy.get_param(rospy.get_name() + '/topics')
        frequency = rospy.get_param(rospy.get_name() + '/frequency')
    except KeyError:
        print "value not set"
        quit()

    rate = rospy.Rate(frequency)
    monitor = Monitor("simple_monitor_python")
    last_xvalues_for_calc = 10

    hz_monitors = {}
    for entry in topics:
        rthz = ROSTopicHz(-1, filter_expr=None)
        print entry['name']
        sub = rospy.Subscriber(entry['name'], rospy.AnyMsg, rthz.callback_hz)
        hz_monitors[entry['name']] = rthz

    start_timestamp = time.time()
    while not rospy.is_shutdown():  # main loop


        for entry in topics:
            n_ = len(hz_monitors[entry['name']].times)
            if n_ < last_xvalues_for_calc:
# TODO was ist wenn keine miS gesendet werden. Dann wird nichts ueberprueft und somit keine
# Fehlermeldung rausgegeben!!! && Wenn bereits 10 narchichten da, aber keine neuen mehr
# ankommen?!?
                if time.time()-start_timestamp <= 10: #nach 10 sekunden wird durchgeschaltet
                    continue
            if n_ == 0:  # no mis received, division by 0 catching
                rospy.logwarn("no new miS for topic %s", entry['name'])
                value = "Topic " + entry['name'] + " sends no data"
                monitor.addValue("no topic", value, "", 0.3, AggregationStrategies.LAST)
                continue
# TODO this is mean overall time. Needs mean over since last seconds
            mean = sum(hz_monitors[entry['name']].times[-last_xvalues_for_calc:]) / last_xvalues_for_calc
            freq = 1. / mean if mean > 0. else 0
            if entry['frequency'] - 0.1*freq > freq:
                rospy.logwarn("Frequency of %s is to low: Expected: %f Actual: %f", entry['name'], entry['frequency'], freq)
                value = "Topic " + entry['name'] + " is on " + str(freq) + "Hz, Expected: " +  str(entry['frequency'])
                monitor.addValue("slow topic", value, "", 0.3, AggregationStrategies.LAST)


        rate.sleep()


if __name__ == '__main__':
    try:
        topicmonitor()
    except rospy.ROSInterruptException:
        pass
