#!/usr/bin/env python
"""
 Author: Michael Lange, Leibniz Universitaet Hannover, 2018

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
#bag_dir = '/home/michael/youbot_local_dev/youbot_rosbag_20180828_szenario1/fahrt3.bag'
#bag_dir = '/home/michael/youbot_local_dev/youbot_rosbag_20180828_szenario2/fahrt1.bag'
bag_dir = '/home/michael/youbot_local_dev/youbot_rosbag_20180830_szenario3/fahrt1.bag'

def init():
    rospy.init_node("monitoring_bag_topic_extract")

def get_bag_data():
    topic_list = []
    bag = rosbag.Bag(bag_dir)
    for topic, msg, t in bag.read_messages():
        if topic == "/monitoring":
            for element in msg.info[0].values:
                if element.key not in topic_list:
                    topic_list.append(element.key)
    for element in topic_list:
        print element

if __name__ == '__main__':
    init()
    get_bag_data()
