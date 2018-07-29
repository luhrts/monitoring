#!/usr/bin/env python

#Author: Michael Lange, Leibniz Universitaet Hannover, 2018


import rospy


from monitoring_core.monitor import Monitor
from nav_msgs.msg import OccupancyGrid

first_map = True
old_map = None

def init():
     rospy.init_node("map_monitor", anonymous=True)

def subscribe():
    rospy.Subscriber("/map", OccupancyGrid, map_monitor)

def map_monitor(msg):
"""
Compare the current map with the last update. Gather data on total changed tiles,
percentage of the map discovered and rewritten tiles.
"""
    global first_map
    global old_map
    map_size = len(msg.data)
    print map_size
    undiscovered_count = 0
    for element in msg.data:
        if element == -1:
            undiscovered_count = undiscovered_count + 1
    undiscovered_percent = ((undiscovered_count*1.0)/map_size)*100
    discovered_percent = 100 - undiscovered_percent
    print str(undiscovered_percent) + "percent undiscovered"
    print str(discovered_percent) + "percent discovered"
    if first_map:
        first_map = False
        old_map = msg.data
        return

    total_changed_tiles = 0
    newly_discovered = 0
    rewritten_tiles = 0
    i = 0
    for element_new in msg.data:
        if element_new != old_map[i]:
            total_changed_tiles = total_changed_tiles + 1
            if old_map[i] == -1:
                newly_discovered = newly_discovered + 1

    if total_changed_tiles != newly_discovered:
        rewritten_tiles = total_changed_tiles - newly_discovered
    print "total tiles changed since last update: " + str(total_changed_tiles)
    print "newly_discovered "+str(newly_discovered)
    print "tiles rewritten: " + str(rewritten_tiles)




    oldMap = msg.data

if __name__ == '__main__':
    init()
    subscribe()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ERROR")
            pass
