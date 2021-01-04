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
node_processes = {}
cpu_threads = []
thread_list = []
shutdown = False
active_nodes = []
config_nodes = []

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
    proc = None

    while not proc:
        try:
            proc = Process(pid)
        except Exception as e:
            pass
        RATE.sleep()

    while not rospy.is_shutdown():
        try:
            if rospy.is_shutdown():
                break
            #temp = Process(pid)
            cpu_percent[pid] = proc.cpu_percent(interval=None)
        except Exception as e:
            pass
        finally:
            if rospy.is_shutdown():
                break
        RATE.sleep()

def check_cpu_percent_in_thread(pid):
    global cpu_percent
    global cpu_threads
    if pid not in cpu_percent.keys():
        cpu_percent[pid] = 0.0
        t = Thread(target=get_cpu_percent_thread, args=(pid,))
        t.start()
        cpu_threads.append(t)

def init():
    """
    Init rospy node
    check for frequency parameter (default = 1Hz)
    check for filter_type (0 = default (list all), 1 = whitelist, 2 = black list)
    Return: frequency and filter_type
    """
    node_black_list = []
    proc_list = []
    if rospy.has_param('monitor_mode'):
        monitor_mode = rospy.get_param('monitor_mode')
    else:
        monitor_mode = 1
    if rospy.has_param('monitoring/frequency'):
        frequency = rospy.get_param('monitoring/frequency')
    else:
        frequency = 10
    if rospy.has_param('node_ressource_monitor/node_list'):
        node_black_list = rospy.get_param('node_ressource_monitor/node_list')
    else:
        node_black_list = []
    if rospy.has_param('node_ressource_monitor/process_list'):
        proc_list = rospy.get_param('node_ressource_monitor/process_list')
    else:
        proc_list = []
    return frequency, filter_type, node_black_list, proc_list


def get_node_pids(node_black_list):
    """
    List all ROS Nodes
    Return: List of pids for each ROS Nodes
    """
    global active_nodes
    rospy.logdebug("GET_NODE_LIST:")
    node_array_temp = get_node_names()
    pids = []
    names = []

    for node in node_array_temp:
        if node in node_black_list:
            continue
        # else we take all nodes
        try:
            node_api = get_api_uri(rospy.get_master(), node)
            if gethostname() not in node_api[2] and gethostbyname(gethostname()) not in node_api[2]:   # only get info for local nodes
                continue # no local node 
            code, msg, pid = ServerProxy(node_api[2]).getPid(ID)
            active_nodes[node] = pid
            pids.append(pid)
            rospy.loginfo("Appending node %s", node_name)
        except Exception as e:
            rospy.logwarn("Couldnt get PID of Node: %s", node)
            rospy.logwarn("Will try it again later")
    rospy.loginfo("=============================")
    return pids


def get_node_thread_list(node_list):
    """
    List all ROS Nodes
    Return: List of threads for each ROS Nodes
    """
    global active_nodes
    rospy.logdebug("GET_NODE_LIST:")
    node_array_temp = get_node_names()
    node_thread_list = []

    for node_name in node_array_temp:
        found = False
        for node in node_list:
            if node in node_name:
                found = True
        if FILTER_TYPE_ == Filter_type.BLACKLIST:
            # dont take node_names that are in node_list
            if found:
                continue
        elif FILTER_TYPE_ == Filter_type.WHITLELIST:
            # only take node_names that are in node_list
            if not found:
                continue
        # else we take all nodes

        active_nodes.append(node_name)
        t = Thread(target=gather_info, args=(node_name,))
        rospy.loginfo("Appending node %s", node_name)
        node_thread_list.append(t)
        
    rospy.loginfo("=============================")
    return node_thread_list

def check_for_new_nodes(timer):
    global active_nodes
    global config_nodes
    global thread_list

    node_array_temp = get_node_names()
    for node_name in node_array_temp:
        if node_name in active_nodes:
            continue

        found = False
        for node in config_nodes:
            if node in node_name:
                found = True
        if FILTER_TYPE_ == Filter_type.BLACKLIST:
            # dont take node_names that are in node_list
            if found:
                continue
        elif FILTER_TYPE_ == Filter_type.WHITLELIST:
            # only take node_names that are in node_list
            if not found:
                continue
        active_nodes.append(node_name)
        t = Thread(target=gather_info, args=(node_name,))
        t.start()
        thread_list.append(t)


def get_process_pids(process_list):
    pids, names = [], []
    for proc in process_list:
        temp = check_output('ps ax | grep '+proc, shell=True).split('\n')
        for val in temp:
            if val.find('grep') == -1 and val:
                pid = val.split(' ')[0]
                if not pid:
                    try:
                        pid = val.split(' ')[1]
                    except Exception as e:
                        pass
                        # we need this except cause ps ax aslo finds the pid of the grep command itself
                name = val.split(proc)[1].split(" ")[0]+proc
                if pid:
                    pids.append(int(pid))
                    names.append(name)
    return [names, pids]


def get_pid_list(filter_string):
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
    Dont use this Function. .as_dict() consumes a lot of the cpu.
    gather all information provided by psutil.Process(pid)
    """
    node_process_info = Process(pid)
    return node_process_info.as_dict()


def gather_info(node_name):
    """
    calls print_to_console_and_monitor for the retrieved node
    """
    #node_list = get_node_list() #+ get_non_ros_process_list("/home/user/external")
    #node_list += get_programm_list()

    got_api_uri = False
    is_local_node = True

    while not rospy.is_shutdown() and is_local_node:
        if not got_api_uri:
            try:
                node_api = get_api_uri(rospy.get_master(), node_name)
                if gethostname() not in node_api[2] and gethostbyname(gethostname()) not in node_api[2]:   # only get info for local nodes
                    is_local_node = False
                    continue # this could be break ??

                code, msg, pid = ServerProxy(node_api[2]).getPid(ID)

                node = NODE(node_name, pid)
                if pid:
                    check_cpu_percent_in_thread(pid)
                rospy.logdebug("Node_name: " + node.name + " Node_PID: " + str(node.pid))
                got_api_uri = True
            except Exception as e:
                RATE.sleep()
                continue
        else:
            try:
                print_to_console_and_monitor(node.name, node.pid)
            except Exception as e:
                if rospy.is_shutdown():
                    break
                rospy.logerr("[NodeResourceMonitor ]Node: %s (pid: %d)", node.name, node.pid)
                rospy.logerr("[NodeResourceMonitor ]%s", str(e))

        if rospy.is_shutdown():
            break
        RATE.sleep()


def print_to_console_and_monitor(name, pid):
    """
    print node values to console
    publish node values to /monitioring topic
    check for filter options
    define DEFAULT options
    """
    global cpu_percent
    global node_processes
    #get all values belonging to pid
    try:
        if pid not in node_processes.keys():
            node_process_info = Process(pid)
            node_processes[pid] = node_process_info
        else:
            node_process_info = node_processes[pid]
    except psutil_error as pe:
        rospy.logerr("No process with pid: " + pid) #THROWS ERROR PLS FIX
        return
    except Exception as e:
        print e
        return

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


def shutdown_cb():
    global threads
    global thread_list
    for t in thread_list:
        t.join()
    print "Thread List terminated"
    for t in cpu_threads:
        t.join()

    print "All Threads terminated"


if __name__ == '__main__':
    #TODO if a node restarte (with a new pid)
    # it is lost for the node_res_mon

    # NEW Approach
    # Input: node_black_list, process_list
    # 1. get pids_list
    # 2. start a process to gather info for each pid
    # 3. one timer to check for new node / update pids
    

    cpu_threads = []
    thread_list = []
    shutdown = False
    cpu_percent = {}
    node_processes = {}
    active_nodes = []
    config_nodes = []

    rospy.init_node("node_ressource_monitor", anonymous=True)
    FREQUENCY, FILTER_TYPE_, node_black_list, process_list = init()
    rospy.loginfo("Frequency: "+str(FREQUENCY))
    RATE = rospy.Rate(FREQUENCY)
    # create MONITOR_ object the node had no name ....
    MONITOR_ = Monitor("node_ressource_monitor")

    pids = get_node_pids(node_black_list)
    pids += get_process_pids(process_list)

    #Needs optimization due to high cpu load
    thread_list = get_node_thread_list(node_list)
    for t in thread_list:
        t.start()
    timer = rospy.Timer(rospy.Duration(5.0), check_for_new_nodes)
    rospy.on_shutdown(shutdown_cb)
    rospy.spin()


