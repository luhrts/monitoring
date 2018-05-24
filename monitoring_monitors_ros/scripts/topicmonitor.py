#!/usr/bin/env python
import rospy
from rostopic import *
from time import sleep
from monitoring_msgs.msg import *
from monitoring_core.monitor import Monitor
import time


def topicmonitor():
    rospy.init_node("topicmonitor")
    topics = rospy.get_param(rospy.get_name() + '/topics')
    frequency = rospy.get_param(rospy.get_name() + '/frequency')
    rate = rospy.Rate(frequency)
    monitor = Monitor("simple_monitor_python")
    
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


        for entry in topics:
            n = len(hzMonitors[entry['name']].times) 
            if(n < lastXvaluesForCalc):  # TODO was ist wenn keine miS gesendet werden. Dann wird nichts ueberprueft und somit keine Fehlermeldung rausgegeben!!! && Wenn bereits 10 narchichten da, aber keine neuen mehr ankommen?!?
                if(time.time()-startTimestamp <=10): #nach 10 sekunden wird durchgeschaltet
                    continue
            if(n == 0):  # no mis received, division by 0 catching
                rospy.logwarn("no new miS for topic %s", entry['name'])
                value = "Topic " + entry['name'] + " sends no data"
                monitor.addValue("no topic", value, "", 0.3)
                continue
            mean = sum(hzMonitors[entry['name']].times[-lastXvaluesForCalc:]) / lastXvaluesForCalc  # TODO this is mean overall time. Needs mean over since last seconds
            freq = 1. / mean if mean > 0. else 0 
            if(entry['frequency'] - 0.1*freq > freq):  
                rospy.logwarn("Frequency of %s is to low: Expected: %f Actual: %f", entry['name'] , entry['frequency'] , freq)
                value = "Topic " + entry['name'] + " is on " + str(freq) + "Hz, Expected: " + str(entry['frequency'])
                monitor.addValue("slow topic", value, "", 0.3)
                
        monitor.publish()
        rate.sleep()


if __name__ == '__main__':
    try:
        topicmonitor()
    except rospy.ROSInterruptException:
        pass
