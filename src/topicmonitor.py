#!/usr/bin/env python
import rospy
from rostopic import * 
from time import sleep


def topicmonitor():
    rospy.init_node("topicmonitor")
    topics = rospy.get_param(rospy.get_name() + '/topics')
    frequency = rospy.get_param(rospy.get_name() + '/frequency')
    rate = rospy.Rate(frequency)
    
    hzMonitors = {}
    for entry in topics:
        rthz = ROSTopicHz(-1, filter_expr=None)
        print entry['name']
        sub = rospy.Subscriber(entry['name'], rospy.AnyMsg, rthz.callback_hz)
        
        hzMonitors[entry['name']] = rthz
              
    while not rospy.is_shutdown():  #main loop
        print '----'
        for entry in topics:
            n = len(hzMonitors[entry['name']].times) 
            if(n==0):   #no msgs received, division by 0 catching
                rospy.logwarn("no new MSGS for topic %s", entry['name'] )
                continue
            mean = sum(hzMonitors[entry['name']].times) / n 
            freq = 1./mean if mean > 0. else 0 
            if(entry['frequency']-0.1 > freq):  #TODO send warning on topic
                rospy.logwarn("Frequency of %s is to low: Expected: %f Actual: %f",entry['name'] ,entry['frequency'] ,freq)
            
        rate.sleep()


if __name__ == '__main__':
    try:
        topicmonitor()
    except rospy.ROSInterruptException:
        pass