#!/usr/bin/env python
"""
 Author: Michael Lange, Leibniz Universitaet Hannover, 2018

 This scripts lists all running rosnodes and displays all values the
 psutil package is providing for each node.
 Some values returned by psutil that contain a dictionary of values will be
 divided into single fields. This will allow the rqt plugin "monitoring_rqt_plot"
 to plot these values.
 Supports a filter function in order to only retrieve information of specific
 nodes with specific values.

"""
import xmlrpclib
import traceback
import rospy
import psutil
import rosnode


from enum import Enum
from monitoring_core.monitor import Monitor


ID = "NODEINFO"
Monitor_ = Monitor("node_ressource_monitor")

class NODE:
        def __init__(self, name, pid):
            self.pid = pid
            self.name = name

class FILTER_TYPE:
    DEFAULT = 0
    WHITLELIST = 1
    BLACKLIST = 2

def init():
    """
    Init rospy node
    check for frequency parameter (default = 1Hz)
    check for filter_type (0 = default (list all), 1 = whitelist, 2 = black list)
    Return: frequency and filter_type
    """
    if rospy.has_param('node_ressource_monitor/frequency'):
        frequency = rospy.get_param('node_ressource_monitor/frequency')
    else:
        frequency = 10
    if rospy.has_param('node_ressource_monitor/filter_type'):
        filter_type = rospy.get_param('node_ressource_monitor/filter_type')
    else:
        filter_type = FILTER_TYPE.DEFAULT
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
        node_list.append(NODE(node_name, pid))
        rospy.loginfo("Node_name: " + node_list[j].name + " Node_PID: " + str(node_list[j].pid))
        j+=1
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
    calls monitor.publish() after iterating over alle nodes in list
    """
    node_list = get_node_list()
    for i in node_list:
        try:
            print_to_console_and_monitor(i.name, i.pid)
            rospy.loginfo("------------------------------")
        except Exception:
            rospy.logerr(traceback.format_exc())
            rospy.logerr("----------NO_SUCH_PROCESS_ERROR---------------")         
    rospy.loginfo("=============================")

def print_to_console_and_monitor(name, pid):
    """
    print node values to console
    publish node values to /monitioring topic
    check for filter options
    define DEFAULT options
    """
    #get all values belonging to pid
    try:
        node_process_info = get_process_info(pid)
    except psutil._exceptions.NoSuchProcess:
        #rospy.logerr("No process with pid: " + pid) #THROWS ERROR PLS FIX
        node_process_info = {} #Need to initialize var in order to prevent error
        
    #check if there is a node with the same name in filter config
    if filter_type == FILTER_TYPE.WHITLELIST:
        if rospy.has_param('/node_ressource_monitor' + name):
            node_value_filter = rospy.get_param('/node_ressource_monitor' + name)
        else:
            rospy.logwarn(name + " has no filter entry")
            return
    #Define DEFAULT values to publish
    if filter_type == FILTER_TYPE.DEFAULT:
        node_value_filter = {'values':['cpu_affinity', 'cpu_percent', 'cpu_times'
        , 'create_time', 'exe', 'io_counters', 'memory_info', 'memory_percent', 'name', 'num_ctx_switches', 'status']}
    #iterate over all keys given for node in node_filter
    for key in node_value_filter.get("values"):
        #if psutil delivers a value for a key, send this value to its dedicated function
        if key in node_process_info.keys():
            if key == "cpu_percent":
                temp = psutil.Process(pid)
                value = temp.cpu_percent(0.1)
            else:
                value = node_process_info.get(key)
            if VALUE_DICT.has_key(key):
                #call dedicated function for key which adds relevant monitoring parameters to
                #/monitoring topic
                VALUE_DICT[key](value, name)

"""
psutil values to monitor functions
"""

def children_to_monitor(value, name):
    rospy.loginfo("children: " + str(value))
    monitor_string = name + "/children"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def cmdline_to_monitor(value, name):
    rospy.loginfo("cmdline: " + str(value))
    monitor_string = name + "/cmdline"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def connections_to_monitor(value, name):
    rospy.loginfo("connections: " + str(value))
    monitor_string = name + "/connections"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def cpu_affinity_to_monitor(value, name):
    rospy.loginfo("cpu_affinity: " + str(value))
    monitor_string = name + "/cpu_affinity"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def cpu_percent_to_monitor(value, name):
    rospy.loginfo("cpu_percent: " + str(value))
    monitor_string = name + "/cpu_percent"
    monitor_value = str(value)
    monitor_unit = "%"
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def cpu_times_to_monitor(value, name):
    rospy.loginfo("cpu_times: " + str(value))
    monitor_string = name + "/cpu_times"
    monitor_value = str(value)
    monitor_unit = "sec"
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def create_time_to_monitor(value, name):
    rospy.loginfo("create_time: " + str(value))
    monitor_string = name + "/create_time"
    monitor_value = str(value)
    monitor_unit = "ms"
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def cwd_to_monitor(value, name):
    rospy.loginfo("cwd: " + str(value))
    monitor_string = name + "/cwd"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def exe_to_monitor(value, name):
    rospy.loginfo("exe-path: " + str(value))
    monitor_string = name + "/exe"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def gids_to_monitor(value, name):
    rospy.loginfo("gids: " + str(value))
    monitor_string = name + "/gids"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def io_counters_to_monitor(value, name):
    rospy.loginfo("io_counters: " + str(value))
    monitor_string = name + "/io_counters"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def ionice_to_monitor(value, name):
    rospy.loginfo("ionice: " + str(value))
    monitor_string = name + "/ionice"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def is_running_to_monitor(value, name):
    rospy.loginfo("is_running: " + str(value))
    monitor_string = name + "/is_running"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def memory_info_to_monitor(value, name):
    """
    added two additional blocks that divide the value string into one field for
    rss and vms.
    """
    rospy.loginfo("memory_info: " + str(value))
    monitor_string = name + "/memory_info"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

    rospy.loginfo("memory_info_rss: " + str(value.rss))
    monitor_string = name + "/memory_info_rss"
    monitor_value = str(value.rss)
    monitor_unit = "byte"
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

    rospy.loginfo("memory_info_vms: " + str(value.vms))
    monitor_string = name + "/memory_info_vms"
    monitor_value = str(value.vms)
    monitor_unit = "byte"
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def memory_info_ex_to_monitor(value, name):
    rospy.loginfo("memory_info_ex: " + str(value))
    monitor_string = name + "/memory_info_ex"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def memory_maps_to_monitor(value, name):
    rospy.loginfo("memory_maps: " + str(value))
    monitor_string = name + "/memory_maps"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def memory_percent_to_monitor(value, name):
    rospy.loginfo("memory_percent: " + str(value))
    monitor_string = name + "/memory_percent"
    monitor_value = str(value)
    monitor_unit = "%"
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def name_to_monitor(value, name):
    rospy.loginfo("name: " + str(value))
    monitor_string = name + "/name"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def nice_to_monitor(value, name):
    rospy.loginfo("nice: " + str(value))
    monitor_string = name + "/nice"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def num_ctx_switches_to_monitor(value, name):
    """
    added two additional blocks that divide the value string into one field for
    voluntary and involuntary context switches.
    """
    rospy.loginfo("num_ctx_switches: " + str(value))
    monitor_string = name + "/num_ctx_switches"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)
    rospy.loginfo("voluntary_ctx_switches: " + str(value.voluntary))
    monitor_string = name + "/num_ctx_switches_voluntary"
    monitor_value = str(value.voluntary)
    monitor_unit = "ctx_switches"
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

    rospy.loginfo("involuntary_ctx_switches: " + str(value.involuntary))
    monitor_string = name + "/num_ctx_switches_involuntary"
    monitor_value = str(value.involuntary)
    monitor_unit = "ctx_switches"
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def num_fds_to_monitor(value, name):
    rospy.loginfo("num_fds: " + str(value))
    monitor_string = name + "/num_fds"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def num_handles_to_monitor(value, name):
    rospy.loginfo("num_handles: " + str(value))
    monitor_string = name + "/num_handles"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def num_threads_to_monitor(value, name):
    rospy.loginfo("num_threads: " + str(value))
    monitor_string = name + "/num_threads"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def open_files_to_monitor(value, name):
    rospy.loginfo("open_files: " + str(value))
    monitor_string = name + "/open_files"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def parent_to_monitor(value, name):
    rospy.loginfo("parent: " + str(value))
    monitor_string = name + "/parent"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def pid_to_monitor(value, name):
    rospy.loginfo("pid: " + str(value))
    monitor_string = name + "/pid"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def ppid_to_monitor(value, name):
    rospy.loginfo("ppid: " + str(value))
    monitor_string = name + "/ppid"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def rlimit_to_monitor(value, name):
    rospy.loginfo("rlimit: " + str(value))
    monitor_string = name + "/rlimit"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def status_to_monitor(value, name):
    rospy.loginfo("status: " + str(value))
    monitor_string = name + "/status"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def terminal_to_monitor(value, name):
    rospy.loginfo("terminal: " + str(value))
    monitor_string = name + "/terminal"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def threads_to_monitor(value, name):
    rospy.loginfo("threads: " + str(value))
    monitor_string = name + "/threads"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def uids_to_monitor(value, name):
    rospy.loginfo("uids: " + str(value))
    monitor_string = name + "/uids"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

def username_to_monitor(value, name):
    rospy.loginfo("username: " + str(value))
    monitor_string = name + "/username"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    Monitor_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl)

"""
This dictionary maps the keys of node_value_filter to the corrseponding
value return functions
"""
VALUE_DICT = {
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
    rospy.init_node('node_ressource_monitor', anonymous=True)
    frequency, filter_type = init()
    rate = rospy.Rate(frequency)

    while not rospy.is_shutdown():
        try:
            gather_info()
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ERROR")
        except Exception as e:
            rospy.logerr(traceback.format_exc())
