#!/usr/bin/env python
'''
   Copyright (c) 2018, University of Hannover
                       Institute for Systems Engineering - RTS
                       Professor Bernardo Wagner
 
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:
 
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.
 
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
''' 
#import xmlrpclib
from xmlrpclib import ServerProxy
#import traceback
from traceback import format_exc
import rospy
#import psutil
from psutil import Process
from psutil import NoSuchProcess as psutil_error
#import rosnode
from rosnode import get_node_names
from rosnode import get_api_uri
#import socket
from socket import gethostbyname
from socket import gethostname
#import subprocess
from subprocess import check_output
#import threading
from threading import Thread

from socket import error as socket_error

from monitoring_core.monitor import Monitor
from monitoring_core.monitor import AggregationStrategies


ID = "NODEINFO"
MONITOR_ = Monitor("node_ressource_monitor")
monitor_mode = 1
cpu_percent = {}
threads = []
shutdown = False

class NODE(object):
    def __init__(self, name, pid):
        self.pid = pid
        self.name = name

class Filter_type(object):
    DEFAULT = 0
    WHITLELIST = 1
    BLACKLIST = 2

def get_cpu_percent_thread(pid):
    global cpu_percent
    global shutdown
    while not shutdown:
        try:
            temp = Process(pid)
            cpu_percent[pid] = temp.cpu_percent(0.3)
        except Exception as e:
            pass

def check_cpu_percent_in_thread(pid):
    global cpu_percent
    global threads
    if pid not in cpu_percent.keys():
        cpu_percent[pid] = 0.0
        t = Thread(target=get_cpu_percent_thread, args=(pid,))
        t.start()
        threads.append(t)

def init():
    """
    Init rospy node
    check for frequency parameter (default = 1Hz)
    check for filter_type (0 = default (list all), 1 = whitelist, 2 = black list)
    Return: frequency and filter_type
    """
    if rospy.has_param('monitor_mode'):
        monitor_mode = rospy.get_param('monitor_mode')
    else:
        monitor_mode = 1
    if rospy.has_param('node_ressource_monitor/frequency'):
        frequency = rospy.get_param('node_ressource_monitor/frequency')
    else:
        frequency = 1
    if rospy.has_param('node_ressource_monitor/filter_type'):
        filter_type = rospy.get_param('node_ressource_monitor/filter_type')
    else:
        filter_type = Filter_type.DEFAULT
    return frequency, filter_type

def get_node_list():
    """
    List all ROS Nodes
    Return: List containing ROS Nodes(name, pid)
    """
    rospy.logdebug("GET_NODE_LIST:")
    node_array_temp = get_node_names()
    node_list = []
    j = 0
    for node_name in node_array_temp:
        try:
            node_api = get_api_uri(rospy.get_master(), node_name)
            if gethostname() not in node_api[2] and gethostbyname(gethostname()) not in node_api[2]:   # Only Get Info for Local Nodes
                continue

            code, msg, pid = ServerProxy(node_api[2]).getPid(ID)

            node_list.append(NODE(node_name, pid))
            check_cpu_percent_in_thread(pid)
            rospy.logdebug("Node_name: " + node_list[j].name + " Node_PID: " + str(node_list[j].pid))
            j += 1
        except socket_error as serr:
             pass
    rospy.logdebug("=============================")
    return node_list

def get_pid_list(base_name):
    temp = check_output('ps ax | grep '+filter_string, shell=True).split('\n')
    pids, names = [], []
    for val in temp:
        if val.find('grep') == -1 and val:
            pid = val.split(' ')[0]
            if not pid:
                try:
                    pid = val.split(' ')[1]
                except Exception as e:
                    pass
                    # we need this except cause ps ax aslo finds the pid of the grep command itself
            name = val.split(base_name)[1].split(" ")[0]
            if pid:
                pids.append(int(pid))
                names.append(name)
    return [names, pids]

def get_non_ros_process_list(base_name):
    program_list = []
    temp = get_pid_list(base_name)
    for idx, val in enumerate(temp[0]):
        program_list.append(NODE(temp[0][idx],temp[1][idx]))
    return program_list

def get_process_info(pid):
    """
    gather all information provided by psutil.Process(pid)
    """
    node_process_info = Process(pid)
    return node_process_info.as_dict()


def gather_info():
    """
    obtains list of all running nodes by calling get_node_list()
    calls print_to_console_and_monitor for each retrieved node
    calls monitor.publish() after iterating over alle nodes in list
    """
    node_list = get_node_list() #+ get_non_ros_process_list("/home/user/external")
    #bw_node_list = get_programm_list()
    #node_list += get_programm_list()
    for i in node_list:
        if rospy.is_shutdown():
            break
        try:
            print_to_console_and_monitor(i.name, i.pid)
        except Exception as e:
            if rospy.is_shutdown():
                break
            rospy.logerr("[NodeResourceMonitor ]Node: %s (pid: %d)", i.name, i.pid)
            rospy.logerr("[NodeResourceMonitor ]%s", str(e))


def print_to_console_and_monitor(name, pid):
    """
    print node values to console
    publish node values to /monitioring topic
    check for filter options
    define DEFAULT options
    """
    global cpu_percent
    #get all values belonging to pid
    try:
        node_process_info = Process(pid)
    except psutil_error as pe:
        #rospy.logerr("No process with pid: " + pid) #THROWS ERROR PLS FIX
        return
    #check if there is a node with the same name in filter config
    if FILTER_TYPE_ == Filter_type.WHITLELIST:
        if rospy.has_param('/node_ressource_monitor' + name):
            node_value_filter = rospy.get_param('/node_ressource_monitor' + name)
        else:
            rospy.logwarn(name + " has no filter entry")
            return
    if FILTER_TYPE_ == Filter_type.BLACKLIST:
        if rospy.has_param('/node_ressource_monitor' + name):
            blacklist_value_= rospy.get_param('/node_ressource_monitor' + name)
            default_value = {'values':['cpu_affinity', 'cpu_percent', 'cpu_times', 'create_time',
                             'exe','io_counters', 'memory_info', 'memory_percent', 'name', 'num_ctx_switches', 'status']}

            for i in blacklist_value_['values']:
                if i == default_value['values']:
                    del default_value['values'].i
            node_value_filter = default_value
        else:
            rospy.logwarn(name + " has no filter entry")
            return
    #Define DEFAULT values to publish
    if FILTER_TYPE_ == Filter_type.DEFAULT:
        node_value_filter = {'values':['cpu_affinity','cpu_percent','cpu_times', 'create_time',
                             'exe','io_counters', 'memory_info', 'memory_percent', 'name', 'num_ctx_switches', 'status']}
    #iterate over all keys given for node in node_filter
    for key in node_value_filter.get("values"):
        try:
            if not VALUE_DICT.has_key(key):
                continue

            if key == "cpu_percent":

                if pid in cpu_percent.keys():
                    value = cpu_percent[pid]
                    VALUE_DICT[key](value, name)
                else:
                    rospy.logwarn("No CPU Percent for Node: %s", name)
                    rospy.logwarn("Setting CPU Percent to 0.0")
                    value = 0.0
            elif hasattr(node_process_info, key):
                method_to_call = getattr(node_process_info, key)
                value = get_value(method_to_call)
                VALUE_DICT[key](value, name)
            elif hasattr(node_process_info, "get_"+key):
                method_to_call = getattr(node_process_info, "get_"+key)
                value = get_value(method_to_call)
                VALUE_DICT[key](value, name)

        except Exception as e:
            rospy.logdebug("Node: %s (%d) - %s not found", name, pid, key)
            pass

def get_value(method_to_call):
    if callable(method_to_call):
        return method_to_call()
    else:
        return method_to_call

"""
psutil values to monitor functions
"""

def children_to_monitor(value, name):
    monitor_string = name + "/children"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def cmdline_to_monitor(value, name):
    monitor_string = name + "/cmdline"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def connections_to_monitor(value, name):
    monitor_string = name + "/connections"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def cpu_affinity_to_monitor(value, name):
    monitor_string = name + "/cpu_affinity"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def cpu_percent_to_monitor(value, name):
    monitor_string = name + "/cpu_percent"
    monitor_value = str(value)
    monitor_unit = "%"
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def cpu_times_to_monitor(value, name):
    for field, value in value._asdict().iteritems():
        monitor_string = name + "/cpu_times_" + str(field)
        monitor_value = str(value)
        monitor_unit = "sec"
        monitor_errorlvl = 0

        MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def create_time_to_monitor(value, name):
    monitor_string = name + "/create_time"
    monitor_value = str(value)
    monitor_unit = "ms"
    monitor_errorlvl = 0


    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def cwd_to_monitor(value, name):
    monitor_string = name + "/cwd"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def exe_to_monitor(value, name):
    monitor_string = name + "/exe"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def gids_to_monitor(value, name):
    monitor_string = name + "/gids"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def io_counters_to_monitor(value, name):
    """
    The for-loop iterates over all fields contained within the named tuple and adds
    the values to the monitoring system
    """
    if value is None:       # Important to filter processes with no io counters (ping_monitor)
        return

    for field, value in value._asdict().iteritems():
        monitor_string = name + "/io_counters_" + str(field)
        monitor_value = str(value)
        if "byte" in field:
            monitor_unit = "bytes"
        else:
            monitor_unit = " "
        monitor_errorlvl = 0

        MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def ionice_to_monitor(value, name):
    monitor_string = name + "/ionice"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def is_running_to_monitor(value, name):
    monitor_string = name + "/is_running"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def memory_info_to_monitor(value, name):
    """
    added two additional blocks that divide the value string into one field for
    rss and vms.
    """
    monitor_string = name + "/memory_info_rss"
    monitor_value = str(value.rss)
    monitor_unit = "byte"
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

    monitor_string = name + "/memory_info_vms"
    monitor_value = str(value.vms)
    monitor_unit = "byte"
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def memory_info_ex_to_monitor(value, name):
    monitor_string = name + "/memory_info_ex"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def memory_maps_to_monitor(value, name):
    monitor_string = name + "/memory_maps"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def memory_percent_to_monitor(value, name):
    monitor_string = name + "/memory_percent"
    monitor_value = str(value)
    monitor_unit = "%"
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def name_to_monitor(value, name):
    monitor_string = name + "/name"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def nice_to_monitor(value, name):
    monitor_string = name + "/nice"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def num_ctx_switches_to_monitor(value, name):
    """
    added two additional blocks that divide the value string into one field for
    voluntary and involuntary context switches.
    """
    monitor_string = name + "/num_ctx_switches_voluntary"
    monitor_value = str(value.voluntary)
    monitor_unit = "ctx_switches"
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

    monitor_string = name + "/num_ctx_switches_involuntary"
    monitor_value = str(value.involuntary)
    monitor_unit = "ctx_switches"
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def num_fds_to_monitor(value, name):
    monitor_string = name + "/num_fds"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def num_handles_to_monitor(value, name):
    monitor_string = name + "/num_handles"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def num_threads_to_monitor(value, name):
    monitor_string = name + "/num_threads"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def open_files_to_monitor(value, name):
    monitor_string = name + "/open_files"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def parent_to_monitor(value, name):
    monitor_string = name + "/parent"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def pid_to_monitor(value, name):
    monitor_string = name + "/pid"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def ppid_to_monitor(value, name):
    monitor_string = name + "/ppid"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def rlimit_to_monitor(value, name):
    monitor_string = name + "/rlimit"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def status_to_monitor(value, name):
    monitor_string = name + "/status"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def terminal_to_monitor(value, name):
    monitor_string = name + "/terminal"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def threads_to_monitor(value, name):
    monitor_string = name + "/threads"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def uids_to_monitor(value, name):
    monitor_string = name + "/uids"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl,monitor_mode)

def username_to_monitor(value, name):
    monitor_string = name + "/username"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0

    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl,monitor_mode)

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
    threads = []
    shutdown = False
    cpu_percent = {}

    rospy.init_node("node_ressource_monitor", anonymous=True)
    FREQUENCY, FILTER_TYPE_ = init()
    rospy.loginfo(FREQUENCY)
    RATE = rospy.Rate(FREQUENCY)
    # create MONITOR_ object the node had no name ....
    MONITOR_ = Monitor("node_ressource_monitor")
    while not rospy.is_shutdown():
        try:
            gather_info()
            RATE.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ERROR")
        except Exception:
            rospy.logerr(format_exc())
    shutdown = True
    for t in threads:
        t.join()

