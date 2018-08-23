#!/usr/bin/env python
import matplotlib.pyplot as plt
import rospy
import rosbag
import sys




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
                        numbers.append(abs(float(kv.value)))
                        timestamps.append(float(t.secs) + float(t.nsecs)/10**9)

        bag.close()

        plt.plot(timestamps, numbers)
        plt.xlabel("Zeit in s")
        plt.ylabel("Strom in A")
        plt.ylim([0, 2])
        plt.show()
            
    else:
        print "First argument is the topic, second is the monitoring key, third is the bag-file!"

