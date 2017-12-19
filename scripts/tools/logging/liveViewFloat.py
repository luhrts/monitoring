#!/usr/bin/env python
import rospy
from rosnode import *
from ros_monitoring.msg import *
import matplotlib.pyplot as plt

msgvalue =""
floatbuffer = []
i =0 

#TODO CLASSE!
def callback(mi):
    
    global msgvalue, floatbuffer
    print floatbuffer
    for kv in mi.values:
        if(kv.key == msgvalue):
            
            floatbuffer.append(kv.value)
         
        

def liveViewFloat(msgkey):
    global msgvalue, floatbuffer, i
    msgvalue = msgkey
    rospy.init_node("floatview")
    sub = rospy.Subscriber('/monitoring/all', MonitoringInfo, callback)
    rate = rospy.Rate(1)
    
    plt.axis([0, 10, 0, 100])
    plt.ion()
    
    while not rospy.is_shutdown():
        #print the value
        for f in floatbuffer:
            plt.scatter(i, float(f))
            i=i+1
            
        plt.pause(0.05)
        floatbuffer = []
        rate.sleep()
        

if __name__ == '__main__':
    try:
        liveViewFloat("CPU Temperatur")
    except rospy.ROSInterruptException:
        pass