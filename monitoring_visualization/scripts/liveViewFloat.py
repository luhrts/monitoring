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
import rosnode 
from monitoring_msgs.msg import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation


#TODO CLASSE!
class LiveFloatGraph(object):

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
        self.ax1 = self.fig.add_subplot(1, 1, 1)
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
            if kv_.key == self.msgvalue:
                self.floatbuffer.append(float(kv_.value))
                self.timestamp.append(float(ma_.info[0].header.stamp.secs
				      + (ma_.info[0].header.stamp.nsecs / 10**9)))
                self.counterlist.append(self.counter)

                self.counter = self.counter+1

if __name__ == '__main__':
    try:
        FG = LiveFloatGraph("Joint 0", 20, 65, 80, 100)

        rospy.init_node("floatview")
        sub = rospy.Subscriber('/monitoring', MonitoringArray, FG.callback)
        RATE = rospy.Rate(1)

        FG.init_plot()
        while not rospy.is_shutdown():
            #print the value
            FG.update_plot()
            RATE.sleep()

    except rospy.ROSInterruptException:
        pass
