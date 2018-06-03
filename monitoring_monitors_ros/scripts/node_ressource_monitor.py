#!/usr/bin/env python
"""
 Author: Michael Lange
 pass the data to the ros monitoring system
 This scripts lists all running rosnodes and displays all values the
 psutil package is providing for each node.
 Supports a filter function in order to only retrieve information of specific
 nodes with specific values.

# TODO: Filter enumration
# TODO: Add units to monitor output
# TODO: nur alle lokalen nodes, wenn whitelist aber auf anderem rechner dann warning
"""


import rospy
import psutil
import rosnode
import xmlrpclib

from std_msgs.msg import String
from monitoring_core.monitor import Monitor
from collections import namedtuple

ID = "NODEINFO"
filter_type = None
monitor = Monitor("node_ressource_monitor")

class node:
        def __init__(self, name, pid):
            self.pid = pid
            self.name = name

def init():
    """
    Init rospy node
    check for frequency parameter (default = 1Hz)
    check for filter_type (0 = default (list all), 1 = whitelist, 2 = black list)
    init monitoring
    Return: frequency and filter_type
    """
    rospy.init_node('node_ressource_monitor', anonymous=True)
    if rospy.has_param('node_ressource_monitor/frequency'):
        frequency = rospy.get_param('node_ressource_monitor/frequency')
    else:
        frequency = 1
    if rospy.has_param('node_ressource_monitor/filter_type'):
        filter_type = rospy.get_param('node_ressource_monitor/filter_type')
    else:
        filter_type = 0
    return frequency, filter_type

def get_node_list():
    """
    List all ROS Nodes
    Return: List containing ROS Nodes(name, pid)
    """
    rospy.loginfo("GET_NODE_LIST:")
    node_array_temp = rosnode.get_node_names()
    node_list = []
    j = 0
    for node_name in node_array_temp:
        node_api = rosnode.get_api_uri(rospy.get_master(), node_name)
        code, msg, pid = xmlrpclib.ServerProxy(node_api[2]).getPid(ID)
        node_list.append(node(node_name, pid))
        rospy.loginfo("Node_name: " + node_list[j].name + " Node_PID: " + str(node_list[j].pid))
        j=j+1
    rospy.loginfo("=============================")
    return node_list

def get_process_info(pid):
    """
    gather all information provided by psutil.Process(pid)
    """
    node_process_info = psutil.Process(pid)
    return node_process_info.as_dict()


def gather_info():
    """
    obtains list of all running nodes by calling get_node_list()
    calls print_to_console_and_monitor for each retrieved node
    """
    node_list = get_node_list()
    for i in node_list:
        try:
            print_to_console_and_monitor(i.name, i.pid)
            rospy.loginfo("------------------------------")
        except psutil._exceptions.NoSuchProcess:
            rospy.logerr("----------NO_SUCH_PROCESS_ERROR---------------")
            pass
    monitor.publish()
    rospy.loginfo("=============================")

def print_to_console_and_monitor(name, pid):
    #get all values belonging to pid
    try:
        node_process_info = get_process_info(pid)
    except psutil._exceptions.NoSuchProcess:
        pass
    #check if there is a node with the same name in filter config
    if rospy.has_param('/node_ressource_monitor' + name):
        node_filter = rospy.get_param('/node_ressource_monitor' + name)
    else:
        rospy.logwarn(name + " has no filter entry")
        return
    #add name of current node to monitor
    monitor_string, monitor_value, monitor_unit, monitor_errorlvl = name_to_monitor(name)
    monitor.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

    #iterate over all keys given for node in node_filter
    for key in node_filter.get("values"):
        #if psutil delivers a value for a key, send this value to its dedicated function
        if key in node_process_info.keys():
            value = node_process_info.get(key)
            if value_dict.has_key(key):
                #call dedicated function for key and retrieve relevant monitoring parameters
                monitor_string, monitor_value, monitor_unit, monitor_errorlvl = value_dict[key](value)
                #add parameters to monitor
                monitor.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)


def children_to_monitor(value):
    rospy.loginfo("children: " + str(value))
    monitor_string = "children"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def cmdline_to_monitor(value):
    rospy.loginfo("cmdline: " + str(value))
    monitor_string = "cmdline"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def connections_to_monitor(value):
    rospy.loginfo("connections: " + str(value))
    monitor_string = "connections"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def cpu_affinity_to_monitor(value):
    rospy.loginfo("cpu_affinity: " + str(value))
    monitor_string = "cpu_affinity"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def cpu_percent_to_monitor(value):
    rospy.loginfo("cpu_percent: " + str(value))
    monitor_string = "cpu_percent"
    monitor_value = str(value)
    monitor_unit = "percent"
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def cpu_times_to_monitor(value):
    rospy.loginfo("cpu_times: " + str(value))
    monitor_string = "cpu_times"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def create_time_to_monitor(value):
    rospy.loginfo("create_time: " + str(value))
    monitor_string = "create_time"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def cwd_to_monitor(value):
    rospy.loginfo("cwd: " + str(value))
    monitor_string = "cwd"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def exe_to_monitor(value):
    rospy.loginfo("exe-path: " + str(value))
    monitor_string = "exe"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def gids_to_monitor(value):
    rospy.loginfo("gids: " + str(value))
    monitor_string = "gids"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def io_counters_to_monitor(value):
    rospy.loginfo("io_counters: " + str(value))
    monitor_string = "io_counters"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def ionice_to_monitor(value):
    rospy.loginfo("ionice: " + str(value))
    monitor_string = "ionice"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def is_running_to_monitor(value):
    rospy.loginfo("is_running: " + str(value))
    monitor_string = "is_running"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def memory_info_to_monitor(value):
    rospy.loginfo("memory_info: " + str(value))
    monitor_string = "memory_info"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def memory_info_ex_to_monitor(value):
    rospy.loginfo("memory_info_ex: " + str(value))
    monitor_string = "memory_info_ex"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def memory_maps_to_monitor(value):
    rospy.loginfo("memory_maps: " + str(value))
    monitor_string = "memory_maps"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def memory_percent_to_monitor(value):
    rospy.loginfo("memory_percent: " + str(value))
    monitor_string = "memory_percent"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def name_to_monitor(value):
    rospy.loginfo("name: " + str(value))
    monitor_string = "name"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def nice_to_monitor(value):
    rospy.loginfo("nice: " + str(value))
    monitor_string = "nice"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def num_ctx_switches_to_monitor(value):
    rospy.loginfo("num_ctx_switches: " + str(value))
    monitor_string = "num_ctx_switches"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def num_fds_to_monitor(value):
    rospy.loginfo("num_fds: " + str(value))
    monitor_string = "num_fds"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def num_handles_to_monitor(value):
    rospy.loginfo("num_handles: " + str(value))
    monitor_string = "num_handles"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def num_threads_to_monitor(value):
    rospy.loginfo("num_threads: " + str(value))
    monitor_string = "num_threads"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def open_files_to_monitor(value):
    rospy.loginfo("open_files: " + str(value))
    monitor_string = "open_files"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def parent_to_monitor(value):
    rospy.loginfo("parent: " + str(value))
    monitor_string = "parent"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def pid_to_monitor(value):
    rospy.loginfo("pid: " + str(value))
    monitor_string = "pid"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def ppid_to_monitor(value):
    rospy.loginfo("ppid: " + str(value))
    monitor_string = "ppid"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def rlimit_to_monitor(value):
    rospy.loginfo("rlimit: " + str(value))
    monitor_string = "rlimit"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def status_to_monitor(value):
    rospy.loginfo("status: " + str(value))
    monitor_string = "status"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def terminal_to_monitor(value):
    rospy.loginfo("terminal: " + str(value))
    monitor_string = "terminal"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def threads_to_monitor(value):
    rospy.loginfo("threads: " + str(value))
    monitor_string = "threads"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def uids_to_monitor(value):
    rospy.loginfo("uids: " + str(value))
    monitor_string = "uids"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

def username_to_monitor(value):
    rospy.loginfo("username: " + str(value))
    monitor_string = "username"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    return monitor_string, monitor_value, monitor_unit, monitor_errorlvl

value_dict = {
  'children': children_to_monitor,
  'cmdline': cmdline_to_monitor,
  'connections': connections_to_monitor,
  'cpu_affinity': cpu_affinity_to_monitor,
  'cpu_percent': cpu_percent_to_monitor,
  'cpu_times': cpu_times_to_monitor,
  'create_time': create_time_to_monitor,
  'cwd': cwd_to_monitor,
  'exe': exe_to_monitor,
  'gids': gids_to_monitor,
  'io_counters': io_counters_to_monitor,
  'ionice': ionice_to_monitor,
  'is_running': is_running_to_monitor,
  'memory_info': memory_info_to_monitor,
  'memory_info_ex': memory_info_ex_to_monitor,
  'memory_maps': memory_maps_to_monitor,
  'memory_percent': memory_percent_to_monitor,
  'name': name_to_monitor,
  'nice': nice_to_monitor,
  'num_ctx_switches': num_ctx_switches_to_monitor,
  'num_fds': num_fds_to_monitor,
  'num_handles': num_handles_to_monitor,
  'num_threads': num_threads_to_monitor,
  'open_files': open_files_to_monitor,
  'parent': parent_to_monitor,
  'pid': pid_to_monitor,
  'ppid': ppid_to_monitor,
  'rlimit': rlimit_to_monitor,
  'status': status_to_monitor,
  'terminal': terminal_to_monitor,
  'threads': threads_to_monitor,
  'uids': uids_to_monitor,
  'username': username_to_monitor}


if __name__ == '__main__':
    frequency, filter_type = init()
    rate = rospy.Rate(frequency)
    rospy.loginfo(frequency)
    while not rospy.is_shutdown():
        try:
            gather_info()
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ERROR")
            pass
