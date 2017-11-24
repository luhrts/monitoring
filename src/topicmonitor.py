#!/usr/bin/env python
import rospy
from rostopic import *
from ros_monitoring.msg import * 
from time import sleep
from help import fillMachineInfo


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
             
    pub = rospy.Publisher('/monitoring/all', MonitoringInfo, queue_size=1)
              
    while not rospy.is_shutdown():  #main loop
        msg = MonitoringInfo()
        msg.header.stamp = rospy.Time.now()
        msg.name = rospy.get_name()
        msg.description = "A Topic-Monitor"
        fillMachineInfo(msg)
        
        for entry in topics:
            n = len(hzMonitors[entry['name']].times) 
            if(n<lastXvaluesForCalc):       #TODO was ist wenn keine MSGS gesendet werden. Dann wird nichts überprüft und somit keine Fehlermeldung rausgegeben!!!
                continue
            if(n==0):   #no msgs received, division by 0 catching
                rospy.logwarn("no new MSGS for topic %s", entry['name'] )
                continue
            mean = sum(hzMonitors[entry['name']].times[-lastXvaluesForCalc:]) / lastXvaluesForCalc #TODO this is mean overall time. Needs mean over since last seconds
            freq = 1./mean if mean > 0. else 0 
            if(entry['frequency']-0.1 > freq):  
                rospy.logwarn("Frequency of %s is to low: Expected: %f Actual: %f",entry['name'] ,entry['frequency'] ,freq)
                kv = KeyValue()
                kv.key = "slow Topic"
                kv.value = "Topic "+entry['name']+" is on "+str(freq)+"Hz, Expected: "+str(entry['frequency'])
                msg.values.append(kv)
                
        if(not len(msg.values) ==0):    #stops publishing if there are no missing nodes
            pub.publish(msg)
        rate.sleep()
        


if __name__ == '__main__':
    try:
        topicmonitor()
    except rospy.ROSInterruptException:
        pass