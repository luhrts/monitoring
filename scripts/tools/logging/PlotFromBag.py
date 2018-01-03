#!/usr/bin/env python
import rospy
import rosbag
import sys
import matplotlib.pyplot as plt



if __name__ == '__main__':
    if(len(sys.argv) == 4):
        bag = rosbag.Bag(sys.argv[1])
        numbers = []
        timestamps = []
        for topic, msg, t in bag.read_messages(topics=[sys.argv[2]]):
            #print msg
            for kv in msg.values:
                if kv.key == sys.argv[3]:
                    numbers.append(kv.value)
                    timestamps.append(t)
            
        bag.close()
        
        plt.plot(numbers)
        plt.ylabel(sys.argv[3])
        plt.show()
            
    else:
        print "First argument is the bag, second is the topic, third is the monitoring key!"