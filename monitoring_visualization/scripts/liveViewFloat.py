#!/usr/bin/env python
import rospy
import rosnode 
from monitoring_msgs.msg import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

#TODO CLASSE!
class LiveFloatGraph: 
  
    def __init__(self, msg, ymin, warnlevel, cirticallevel, ymax):
        self.msgvalue = msg
        self.floatbuffer = []
        self.timestamp = []
        self.counter = 0
        self.counterlist = []
        self.min = ymin
        self.max = ymax
        self.warning = warnlevel
        self.critical = cirticallevel

    def init_plot(self):
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(1,1,1)
        self.axes = plt.gca()
   
    def update_plot(self):
        ani = animation.FuncAnimation(self.fig, self.animate, interval=1000)
        plt.show()

    def animate(self, i):
        self.ax1.clear()
        self.ax1.plot(self.timestamp, self.floatbuffer)
        #self.axes.set_ylim([self.min,self.max])
        #plt.axhspan(self.warning, self.critical, color='yellow', alpha=0.3)
        #plt.axhspan(self.critical, self.max, color='red', alpha=0.3)
        
    def callback(self, ma_):
#         print self.floatbuffer
        for kv_ in ma_.info[0].values:
            if(kv_.key == self.msgvalue):
                self.floatbuffer.append(float(kv_.value))
                self.timestamp.append(float(ma_.info[0].header.stamp.secs + 
		(ma_.info[0].header.stamp.nsecs / 10**9)))
                self.counterlist.append(self.counter)
                
                self.counter = self.counter+1

if __name__ == '__main__':
    try:
        fg = LiveFloatGraph("Joint 0", 20, 65, 80, 100)
        
        rospy.init_node("floatview")
        sub = rospy.Subscriber('/monitoring', MonitoringArray, fg.callback)
        rate = rospy.Rate(1)
        
        fg.init_plot()
        while not rospy.is_shutdown():
            #print the value
            fg.update_plot()
            rate.sleep()
        
    except rospy.ROSInterruptException:
        pass
