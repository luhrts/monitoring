#!/usr/bin/env python
import rospy
import rosbag
import sys
import matplotlib.pyplot as plt



if __name__ == '__main__':
    if(len(sys.argv) == 4) or (len(sys.argv) == 6):
        bag = rosbag.Bag(sys.argv[3])
        numbers = []
        timestamps = []
        for topic, msg, t in bag.read_messages(topics=[sys.argv[1]]):
            #print msg
            for info in msg.info:
                for kv in info.values:
                    if kv.key == sys.argv[2]:
                        numbers.append(kv.value)
                        timestamps.append(t)
            
        bag.close()
        
        plt.plot(numbers)
        plt.ylabel(sys.argv[3])
        plt.ylim([-2,0])
        plt.show()
            
    else:
        print "First argument is the topic, second is the monitoring key, third is the bag-file!"