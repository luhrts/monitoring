#!/usr/bin/env python

#Author: Michael Lange, Leibniz Universitaet Hannover, 2018


import rospy


from monitoring_core.monitor import Monitor
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from math import hypot
from tf import TransformListener

monitor = Monitor("marker_monitor")
marker_dict = {"seen_markers":[],"assured_markers":[]}
seen_marker_dict = {}
assured_marker_dict = {}
seen_ids = []
seen_assured_ids = []
tf = None
seen_pub = None
assured_pub = None
marker_count = 0

class extended_marker:
    def __init__(self, marker, bot_dist):
        self.marker = marker
        self.bot_dist = bot_dist

def init():
     rospy.init_node("marker_monitor", anonymous=False)

def subscribe():
    global tf
    rospy.Subscriber("/seen_markers", MarkerArray, seen_markers_callback)
    rospy.Subscriber("/assured_markers", MarkerArray, assured_markers_callback)
    tf = TransformListener()

def publish():
    global seen_pub
    global assured_pub
    seen_pub = rospy.Publisher('seen_markers_monitor', MarkerArray, queue_size=100)
    assured_pub = rospy.Publisher('assured_markers_monitor', MarkerArray, queue_size=100)

def seen_markers_callback(msg):
    global seen_marker_dict
    global marker_count
    for element in msg.markers:
        if str(element.id) not in seen_marker_dict.keys():
            markerArrray = MarkerArray()
            seen_marker_dict[str(element.id)] = markerArrray
        element.header.stamp = rospy.Time(0)
        element.lifetime = rospy.Time(10000)
        id = element.id
        element.id = marker_count
        seen_marker_dict[str(id)].markers.append(element)
        marker_count = marker_count + 1

def assured_markers_callback(msg):
    global assured_marker_dict
    global marker_count
    for element in msg.markers:
        if str(element.id) not in assured_marker_dict.keys():
            markerArrray = MarkerArray()
            assured_marker_dict[str(element.id)] = markerArrray
        element.header.stamp = rospy.Time(0)
        element.lifetime = rospy.Time(10000)
        id = element.id
        element.id = marker_count
        assured_marker_dict[str(id)].markers.append(element)
        marker_count = marker_count + 1

def marker_monitor():
    global monitor
    ready_to_publish = False
    for key in seen_marker_dict.keys():
        x = []
        y = []
        for element in seen_marker_dict[key].markers:
            x.append(element.pose.position.x)
            y.append(element.pose.position.y)
        avg_x = sum(x)/(len(x)*1.0)
        avg_y = sum(y)/(len(y)*1.0)
        distance = []
        for element in seen_marker_dict[key].markers:
            distance.append(hypot(avg_x - element.pose.position.x,avg_y - element.pose.position.y))
        avg_distance = sum(distance)/(len(distance)*1.0)
        monitor.addValue("Marker_" + key + "/mean_abs_dev_distance",str(avg_distance),"m",0)
        print "Marker ID: "+ key + "mean_abs_dev_distance"+ str(avg_distance)
        ready_to_publish = True

    if ready_to_publish:
        monitor.publish()

if __name__ == '__main__':
    init()
    subscribe()
    publish()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            #print seen_marker_dict.items()
            for element in seen_marker_dict.keys():
                seen_pub.publish(seen_marker_dict[element])
            for element in assured_marker_dict.keys():
                assured_pub.publish(assured_marker_dict[element])
            marker_monitor()
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ERROR")
            pass
