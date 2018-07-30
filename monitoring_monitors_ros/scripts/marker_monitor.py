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
seen_ids = []
seen_assured_ids = []
tf = None

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

def seen_markers_callback(msg):
    global seen_ids
    for element in msg.markers:
        if element.id not in seen_ids:
            seen_ids.append(element.id)
            tf.waitForTransform("map", "arm_link_5", rospy.Time(0), rospy.Duration(10.0))
            trans, rot = tf.lookupTransform("map","arm_link_5", rospy.Time(0))
            bot_dist = hypot(trans[0] - element.pose.position.x, trans[1] - element.pose.position.y)
            temp = extended_marker(element,bot_dist)
            marker_dict['seen_markers'].append(temp)


def assured_markers_callback(msg):
    global seen_assured_ids
    for element in msg.markers:
        if element.id not in seen_assured_ids:
            seen_assured_ids.append(element.id)
            tf.waitForTransform("map", "arm_link_5", rospy.Time(0), rospy.Duration(10.0))
            trans, rot = tf.lookupTransform("map","arm_link_5", rospy.Time(0))
            bot_dist = hypot(trans[0] - element.pose.position.x, trans[1] - element.pose.position.y)
            temp = extended_marker(element,bot_dist)
            marker_dict['assured_markers'].append(temp)

def marker_monitor():
    global monitor
    ready_to_publish = False
    for element_seen in marker_dict["seen_markers"]:
        for element_assured in marker_dict["assured_markers"]:
            if element_seen.marker.id == element_assured.marker.id:
                distance = hypot(element_seen.marker.pose.position.x - element_assured.marker.pose.position.x, element_seen.marker.pose.position.y - element_assured.marker.pose.position.y)
                print 'Marker ID: ' + str(element_seen.marker.id) + ' Distance between seen and assured pos: ' + str(distance) + " m"
                print 'Marker ID: ' + str(element_seen.marker.id) + ' Distance between Bot and Marker seen: ' + str(element_seen.bot_dist) + " m"
                print 'Marker ID: ' + str(element_assured.marker.id) + ' Distance between Bot and Marker assured: ' + str(element_assured.bot_dist) + " m"
                monitor.addValue("Marker_ID_" + str(element_seen.marker.id) + "_" + "dist_between_seen_assured",str(distance), "m",0)
                monitor.addValue("Marker_ID_" + str(element_seen.marker.id) + "_" + "dist_between_seen_bot",str(element_seen.bot_dist), "m",0)
                monitor.addValue("Marker_ID_" + str(element_seen.marker.id) + "_" + "dist_between_assured_bot",str(element_assured.bot_dist), "m",0)
                ready_to_publish = True
    if ready_to_publish:
        monitor.publish()
        print "publishing"

if __name__ == '__main__':
    init()
    subscribe()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            marker_monitor()
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ERROR")
            pass
