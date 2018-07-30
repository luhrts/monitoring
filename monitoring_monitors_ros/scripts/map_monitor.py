#!/usr/bin/env python

#Author: Michael Lange, Leibniz Universitaet Hannover, 2018


import rospy


from monitoring_core.monitor import Monitor
from nav_msgs.msg import OccupancyGrid

first_SLAM_map = True
old_SLAM_map = None
first_global_map = True
old_global_map = None
first_explorationPlanner_map = True
old_explorationPlanner_map = None
monitor = Monitor("map_monitor")

def init():
     rospy.init_node("map_monitor", anonymous=False)

def subscribe():
    rospy.Subscriber("/map", OccupancyGrid, SLAM_map_callback)
    rospy.Subscriber("/ExplorationPlanner_GridMap", OccupancyGrid, explorationPlanner_map_callback)
    rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, global_map_callback)

def explorationPlanner_map_callback(msg):
    global first_explorationPlanner_map
    global old_explorationPlanner_map
    map_name = 'explorationPlanner_map'
    old_explorationPlanner_map = map_monitor(msg, map_name, first_explorationPlanner_map, old_explorationPlanner_map)
    if first_explorationPlanner_map:
        first_explorationPlanner_map = False

def global_map_callback(msg):
    global first_global_map
    global old_global_map
    map_name = 'global_map'
    old_global_map = map_monitor(msg, map_name, first_global_map, old_global_map)
    if first_global_map:
        first_global_map = False

def SLAM_map_callback(msg):
    global first_SLAM_map
    global old_SLAM_map
    map_name = 'SLAM_map'
    old_SLAM_map = map_monitor(msg, map_name, first_SLAM_map, old_SLAM_map)
    if first_SLAM_map:
        first_SLAM_map = False

def map_monitor(msg, map_name, first_map, old_map):
    """
    Compare the current map with the last update. Gather data on total changed tiles,
    percentage of the map discovered and rewritten tiles.
    """
    map_size = len(msg.data)
    #print map_size
    #print len(old_map) + ""+ map_name
    if first_map:
        first_map = False
        old_map = msg.data
        return old_map
    l = len(old_map)
    if map_size != l:
        rospy.logerr("KACKE")
        return None

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
    print map_name + "/total tiles changed since last update: " + str(total_changed_tiles)
    print map_name + "/newly_discovered "+str(newly_discovered)
    print map_name + "/tiles rewritten: " + str(rewritten_tiles)
    monitor.addValue(map_name+"/total_changed_tiles",str(total_changed_tiles), "", 0)
    monitor.addValue(map_name+"/newly_discovered_tiles",str(newly_discovered), "", 0)
    monitor.addValue(map_name+"/rewritten_tiles",str(rewritten_tiles), "", 0)

    monitor.publish()

    oldMap = msg.data
    print str(len(oldMap)) +""+ map_name
    return oldMap

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
