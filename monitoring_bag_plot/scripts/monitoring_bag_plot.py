#!/usr/bin/env python
"""
 Author: Michael Lange, Leibniz Universitaet Hannover, 2018

 TODO:
 - implement argument for all plots in one figure
 - implement argparse for user friendliness
"""

import rospy
import rosnode
import csv
import datetime
import rosbag
import sys
import matplotlib.pyplot as plt

from math import hypot
from monitoring_msgs.msg import *
from std_msgs.msg import Float64

start_time = None
value_dict = {}

def init():
    rospy.init_node("monitoring_bag_plot")
    for argument in sys.argv:
        value_dict[argument] = {'timestamp':[], 'value':[], 'unit':[]}
    print value_dict
    get_bag_data()
    plot()

def get_bag_data():
    global start_time
    global value_dict
    bag = rosbag.Bag('/home/michael/youbot_local_dev/youbot_rosbag_20180726_erstefahrt/fahrt3.bag')
    init = True
    for topic, msg, t in bag.read_messages():
        if init:
            start_time = t
            init = False
        for argument in sys.argv:
            if topic == "/monitoring":
                for element in msg.info[0].values:
                    if element.key == argument:

                        temp_time = t - start_time
                        temp_time = temp_time.to_sec()
                        value_dict[argument]['timestamp'].append(str(temp_time))
                        value_dict[argument]['value'].append(str(element.value))
                        value_dict[argument]['unit'].append(str(element.unit))

def plot():
    i = 1
    for key in value_dict.keys():
        #plt.figure(i)
        plt.plot(value_dict[key]['timestamp'],value_dict[key]['value'])
        plt.title(key)
        plt.xlabel("time in sec")
        if not value_dict[key]['unit']:
            plt.ylabel("")
        else:
            plt.ylabel(value_dict[key]['unit'][0])
        i = i + 1
    plt.show()

if __name__ == '__main__':
    init()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ERROR")
            pass
    rospy.logout("node - shutdown, writing to file")
    #plt.plot(value_dict["overall cpu load"]['timestamp'],value_dict["overall cpu load"]['value'])
    #plt.show()
"""
with open("ping_data"+ str(datetime.datetime.now().strftime("%y-%m-%d-%H-%M-%S"))+".csv", "w") as file:
        i = 0
        file.write("timestamp,ping ms \n")
        for data in node_dict['Ping']:
            file.write(str(node_dict['timestamp'][i]))
            file.write(",")
            file.write(str(data))
            file.write("\n")
            i = i + 1

"""
