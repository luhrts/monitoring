#!/usr/bin/env python
# Author: Michael Lange
# Dieses Skript listet alle aktiven ROS-Nodes auf,
# dann pro Node Name, exe-path, CPU auslastung [%], MEM auslastung [%]
import rospy
import psutil
import rosnode
import xmlrpclib

from std_msgs.msg import String

ID = '/NODEINFO'

def init():
    
    rospy.init_node('allNodeInfo', anonymous=True)
    
def get_all_nodes():
    #Auflisten alle ROS Nodes
    #Return: Liste aller ROS Nodes
    rospy.loginfo("GET_ALL_NODES")
    nodeArray = rosnode.get_node_names()
    for i in nodeArray:
        rospy.loginfo("Node-name: " + i)    
    rospy.loginfo("=============================")
    return nodeArray

def get_exe_path(node_name):
    #Bestimmt den exe_path einer ROS Node
    node_api = rosnode.get_api_uri(rospy.get_master(), node_name)
    code, msg, pid = xmlrpclib.ServerProxy(node_api[2]).getPid(ID)
    p = psutil.Process(pid)
    rospy.loginfo("Exe-path:  " + p.exe())

def get_cpu_percent(node_name):
    #Bestimmt die CPU Auslastung einer ROS Node
    node_api = rosnode.get_api_uri(rospy.get_master(), node_name)
    code, msg, pid = xmlrpclib.ServerProxy(node_api[2]).getPid(ID)
    p = psutil.Process(pid)
    rospy.loginfo("CPU usage: " + str(p.cpu_percent()/psutil.cpu_count()) + "%")
    
def get_memory_percent(node_name):
    #Bestimmt den RAM Verbrauch einer ROS Node
    node_api = rosnode.get_api_uri(rospy.get_master(), node_name)
    code, msg, pid = xmlrpclib.ServerProxy(node_api[2]).getPid(ID)
    p = psutil.Process(pid)
    rospy.loginfo("RAM usage: " + str(p.memory_percent()) + "%")   

def get_info():
    nodeArray = get_all_nodes()
    for i in nodeArray:
        try:
            rospy.loginfo("Node-name: " + i)
            get_exe_path(i)
            get_cpu_percent(i)
            get_memory_percent(i)
            rospy.loginfo("------------------------------")
        except psutil._exceptions.NoSuchProcess:
            rospy.loginfo("----------^ERROR^---------------")
            pass
        
    rospy.loginfo("=============================")
    
if __name__ == '__main__':
    init()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            get_info()
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ERROR")
            pass
