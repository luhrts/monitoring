#!/usr/bin/env python
import rospy
from rostopic import *
from ros_monitoring.msg import * 
from time import sleep
from help import fillMachineInfo
import time


def topicmonitor():
    rospy.init_node("topicmonitor")
    topics = rospy.get_param(rospy.get_name() + '/topics')
    frequency = rospy.get_param(rospy.get_name() + '/frequency')
    rate = rospy.Rate(frequency)
    
    lastXvaluesForCalc = 10
    
    hzMonitors = {}
    for entry in topics:
        rthz = ROSTopicHz(-1, filter_expr=None)
        print entry['name']
        sub = rospy.Subscriber(entry['name'], rospy.AnyMsg, rthz.callback_hz)
        
        hzMonitors[entry['name']] = rthz
             
    pub = rospy.Publisher('/monitoring', MonitoringArray, queue_size=1)
    startTimestamp = time.time()
    while not rospy.is_shutdown():  # main loop
        ma = MonitoringArray()
        mi = MonitoringInfo()
        ma.info.append(mi)
        mi.header.stamp = rospy.Time.now()
        mi.name = rospy.get_name()
        mi.description = "A Topic-Monitor"
        fillMachineInfo(mi)
        print
        for entry in topics:
            n = len(hzMonitors[entry['name']].times) 
            if(n < lastXvaluesForCalc):  # TODO was ist wenn keine miS gesendet werden. Dann wird nichts ueberprueft und somit keine Fehlermeldung rausgegeben!!! && Wenn bereits 10 narchichten da, aber keine neuen mehr ankommen?!?
                if(time.time()-startTimestamp <=10): #nach 10 sekunden wird durchgeschaltet
                    continue
            if(n == 0):  # no mis received, division by 0 catching
                rospy.logwarn("no new miS for topic %s", entry['name'])
                kv = KeyValue()
                kv.key = "no topic"
                kv.value = "Topic " + entry['name'] + " sends no data"
                mi.values.append(kv)
                continue
            mean = sum(hzMonitors[entry['name']].times[-lastXvaluesForCalc:]) / lastXvaluesForCalc  # TODO this is mean overall time. Needs mean over since last seconds
            freq = 1. / mean if mean > 0. else 0 
            if(entry['frequency'] - 0.1 > freq):  
                rospy.logwarn("Frequency of %s is to low: Expected: %f Actual: %f", entry['name'] , entry['frequency'] , freq)
                kv = KeyValue()
                kv.key = "slow Topic"
                kv.value = "Topic " + entry['name'] + " is on " + str(freq) + "Hz, Expected: " + str(entry['frequency'])
                mi.values.append(kv)
                
        if(not len(mi.values) == 0):  # stops publishing if there are no missing nodes
            pub.publish(ma)
        rate.sleep()


if __name__ == '__main__':
    try:
        topicmonitor()
    except rospy.ROSInterruptException:
        pass
