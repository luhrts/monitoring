#!/usr/bin/env python
"""
 Author: Michael Lange, Leibniz Universitaet Hannover, 2018

 TODO:
 - implement argparse for user friendliness
 - export all data to individual .csv files in individual folder
"""

import rospy
import rosnode
import csv
import datetime
import rosbag
import sys
import matplotlib.pyplot as plt
import argparse

from math import hypot
from monitoring_msgs.msg import *
from std_msgs.msg import Float64

start_time = None
value_dict = {}
combine = False
search_for_key = False
bag_dir = '/home/michael/youbot_local_dev/youbot_rosbag_20180726_erstefahrt/fahrt3.bag'
#bag_dir = '/home/michael/youbot_local_dev/youbot_rosbag_20180809_firstbatterydrive/fahrt1.bag'

def init():
    rospy.init_node("monitoring_bag_plot")
    global combine
    global search_for_key
    for argument in sys.argv:
        if argument == sys.argv[0]:
            continue
        if argument == "-c":
            combine = True
            continue
        if argument == "-s":
            search_for_key = True
            break
        value_dict[argument] = {'timestamp':[], 'value':[], 'unit':[]}
    get_bag_data()
    plot()

def get_bag_data():
    global start_time
    global value_dict
    bag = rosbag.Bag(bag_dir)
    init = True
    for topic, msg, t in bag.read_messages():
        if init:
            start_time = t
            init = False
        if search_for_key:
            if topic == "/monitoring":
                for element in msg.info[0].values:
                    for argument in sys.argv:
                        if argument in element.key and element.key not in value_dict.keys():
                            value_dict[element.key] = {'timestamp':[], 'value':[], 'unit':[]}
        for key in value_dict.keys():
            if topic == "/monitoring":
                for element in msg.info[0].values:
                    if element.key == key:
                        temp_time = t - start_time
                        temp_time = temp_time.to_sec()
                        value_dict[key]['timestamp'].append(str(temp_time))
                        value_dict[key]['value'].append(str(element.value))
                        value_dict[key]['unit'].append(str(element.unit))
def plot():
    i = 1
    legends = []
    plt.style.use('ggplot')
    plt.tick_params(colors = 'black')
    for key in value_dict.keys():
        if not value_dict[key]['value'][0].replace('.','',1).isdigit():
            continue
        if not combine:
            plt.figure(i)
            plt.plot(value_dict[key]['timestamp'],value_dict[key]['value'])
            plt.title(key)
            plt.xlabel("time in sec")
            plt.grid(True)
            if not value_dict[key]['unit']:
                plt.ylabel("")
            else:
                plt.ylabel(value_dict[key]['unit'][0])
        else:
            plt.plot(value_dict[key]['timestamp'],value_dict[key]['value'])
            plt.xlabel("time in sec")
            legends.append(key)
            plt.legend(legends)
            plt.grid(True)
        i = i + 1

    plt.show()

if __name__ == '__main__':
    init()
    """
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
