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

import rospy
import rosnode
import rosbag
import sys
import matplotlib.pyplot as plt
import matplotlib.axis as ax
import math
import numpy

from monitoring_msgs.msg import *
from collections import OrderedDict
from quality_criterion import quality_criterion
from matplotlib.patches import Circle
from matplotlib.offsetbox import (TextArea, DrawingArea, AnnotationBbox)
from matplotlib.font_manager import FontProperties
from copy import copy
from matplotlib.ticker import FormatStrFormatter, ScalarFormatter
import yaml as yaml

start_time = None
value_dict = {}
combine = False

legends = []
lines = []
legend = None
key_list = None
lined = {}


def parse_sys_args():
    if len(sys.argv) > 1:
        path = sys.argv[1]
        spath = sys.argv[2]
        return path, spath
    else:
        print "required 2 args"
        print "1 Arg:   /Path/for/loading/bag_file.bag"
        print "2 Arg: path/to/safe/pictures.png"
        return None, None

def init():
    bpath, spath = parse_sys_args()
    if not bpath or not spath:
        return

    name = bpath.split("/")[-1]
    name = '_' + name.split(".")[0]

    intervention_times, data = get_bag_data(bpath)
    for node in data.keys():
      node_dict = data[node]
      cpu_vals_list, time_list, unit_list = node_dict["cpu_percent"]
      print ("AVG CPU: %f",sum(cpu_vals_list)/len(cpu_vals_list))
      
    #quality_criterion_function(quality_criterion_)
    sorted_infos, scale = presort(data)
    new_plot(spath, sorted_infos, intervention_times, scale, name)
    #plot(photo_path,blacklist, debug_mode_, intervention_times)

def presort(data):
    sorted_d = {}

    io_counter_bytes_l = []
    io_counter_count_l = []
    ctx_switches_l = []
    memory_l = []
    times_l = []
    key_list = ['cpu_percent','cpu_times_user', 'cpu_times_system','io_counters_read_count', 'io_counters_write_count','io_counters_read_bytes','io_counters_write_bytes', 'memory_info_rss', 'memory_info_vms', 'num_ctx_switches_voluntary', 'num_ctx_switches_involuntary']
    units = [' ', 'ctx_switches', '%', 'byte', 'bytes', 'ms', 'sec']
    for node in data.keys():
        if node not in sorted_d.keys():
            sorted_d[node] = {}

        for val in data[node].keys():
            key = val.replace(node, "")
            if key not in key_list:
                continue
            if key not in sorted_d[node].keys():
                sorted_d[node][key] = {'values':[], 'times':[], 'unit':''}

            datas = data[node][val][0]
            times = data[node][val][1]
            nd = []
            # for some keys calculate value per sec
            if "io_counters" in key or 'ctx_switches' in key:
                #print key
                #print len(datas)
                #print len(times)
                #print '\n'
                for i in range(len(datas)):
                    if i == 0:
                        continue

                    dd = datas[i] - datas[i - 1]
                    dt = times[i] - times[i - 1]

                    if dt == 0:
                        print "Data_i: "+str(datas[i])
                        print "Data_-1: "+str(datas[i-1])
                        print "time_i: "+str(times[i])
                        print "time_-1: "+str(times[i-1])
                        print node
                        print key
                        print "\n"
                        nd.append(0.0)
                    else:
                        nd.append(dd / dt)
                times = times[1:]
            else:
                nd = datas

            sorted_d[node][key]['values'] = nd
            sorted_d[node][key]['times'] = times
            #print data[node][val]
            unit = data[node][val][1][0]

            min_, max_ = min(nd), max(nd)
            if 'cpu_times' in key:
                times_l.append(max_ - min_)
                unit = "sec"
            elif 'io_counters' in key:
                if 'byte' in key:
                    io_counter_bytes_l.append(max_ - min_)
                    unit = 'byte / s'
                else:
                    io_counter_count_l.append(max_ - min_)
                    unit = '1 / sec'
            elif 'ctx_switches' in key:
                ctx_switches_l.append(min_ - max_)
                unit = "1 / sec"
            elif 'memory' in key:
                memory_l.append(max_)
                unit = 'byte'
            elif "cpu_percent" in key:
                unit='%'
            sorted_d[node][key]['unit'] = unit

    # now lets calculate a scale using the lists
    mean_times = numpy.mean(times_l)
    mean_bytes = numpy.mean(io_counter_bytes_l)
    mean_count = numpy.mean(io_counter_count_l)
    mean_ctx = numpy.mean(ctx_switches_l)
    mean_ram = numpy.mean(memory_l)
    scale_d = {'cpu_times':mean_times, 'io_bytes':mean_bytes,
               'io_count':mean_count,'ctx_switches':mean_ctx, 'ram':mean_ram,
               'cpu_percent': 50.0}
    return sorted_d, scale_d



def quality_criterion_function(quality_criterion_):
    for argument in value_dict.keys():
        ##print value_dict[argument].keys()
        for value_name in quality_criterion_.keys():
          ##  print value_name
            for argument_name in value_dict[argument].keys():
                if value_name in argument_name:
                    time_float = map(
                        float, value_dict[argument][argument_name]['timestamp'])
                    data_float = map(
                        float, value_dict[argument][argument_name]['value'])
                    judge_func = quality_criterion(
                        time_float, data_float, quality_criterion_[value_name])
                    if judge_func.result == "error":
                        value_dict[argument][argument_name]['errorlevel'] = 'r'
                    if judge_func.result == "warn":
                        value_dict[argument][argument_name]['errorlevel'] = 'gold'
                    if judge_func.result == "ok":
                        value_dict[argument][argument_name]['errorlevel'] = 'g'
                    value_dict[argument][argument_name]['criterion_value'] = judge_func.criterion_value_dict
                   ## print value_dict[argument][argument_name]['criterion_value']

def get_bag_data(bag_dir):
    global start_time
    global value_dict
    start_time = None
    bag = rosbag.Bag(bag_dir)
    inter_times = []
    data_dict = {}
    key_list = ['cpu_percent','cpu_times_user', 'cpu_times_system','io_counters_read_count', 'io_counters_write_count','io_counters_read_bytes','io_counters_write_bytes', 'memory_info_rss', 'memory_info_vms', 'num_ctx_switches_voluntary', 'num_ctx_switches_involuntary']

    for topic, msg, t in bag.read_messages():
        if not start_time:
            start_time = t.to_sec()

        if t.to_sec() - start_time < 0.0:
            start_time = t.to_sec()

        if not topic == "/monitoring":
            continue

        if "node_ressource_monitor" in msg.info[0].description:
            time = msg.header.stamp.to_sec()
            if time - start_time < 0.0:
                start_time = time
            for value in msg.info[0].values:
                dt = time - start_time

                temp = value.key
                if temp.startswith("/"):
                    temp = temp[1:]
                revert = temp[::-1]
                node = revert[revert.find("/"):][::-1]
                key = temp.split("/")[-1]

                if node not in data_dict.keys():
                    data_dict[node] = {}
                if key not in key_list:
                    continue
                if key not in data_dict[node].keys():
                    data_dict[node][key] = ([],[],[])
                try:
                    data_dict[node][key][0].append(float(value.value))
                    data_dict[node][key][1].append(dt)
                    if not data_dict[node][key][2]:
                        data_dict[node][key][2].append(value.unit)
                except Exception as e:
                    pass

    return inter_times, data_dict

def new_plot(photo_path, sorted_infos, intervention_times, scale, s_name):
    i = 1
    global legend
    global fig, ax
    global lined

    for node in sorted_infos.keys():
        plt.figure(i)
        plt.figure(i).suptitle(node)
        plt.tick_params(colors = 'black')
        size_of_plt = math.sqrt(6)
        ab = []
        ax = []
        p = []
        # Need to make 6 Plots
        # cpu percent, cpu_times, ctx_switches, ram info, io_bytes, io_counts
        #key_list = ['cpu_percent','cpu_times_user', 'cpu_times_system','io_counters_read_count', 'io_counters_write_count','io_counters_read_bytes','io_counters_write_bytes', 'memory_info_rss', 'memory_info_vms', 'num_ctx_switches_voluntary', 'num_ctx_switches_involuntary']
        #sorted_d[node][key] = {'values':[], 'times':[], 'unit':''}
        #scale_d = {'cpu_times', 'io_bytes', 'io_count','ctx_switches', 'ram', 'cpu_percent'}
        for j in range(6):
            ax.append(plt.subplot(2, 3, j+1))
            ax[j].yaxis.offsetText.set_fontsize(4)
            ax[j].yaxis.set_major_locator(plt.MaxNLocator(8))
            ax[j].xaxis.set_major_locator(plt.MaxNLocator(8))
            plot_datas = []
            key_s = ""
            if j == 0:
                #cpu percent
                if 'cpu_percent' not in sorted_infos[node].keys():
                    temp = {'values': [0],'times':[0],'unit':''}
                else:
                    temp = sorted_infos[node]['cpu_percent']
                plot_datas.append((temp['values'],temp['times'],temp['unit']))
                key_s = 'cpu_percent'
                legend_ = ['cpu usage']
            elif j == 1:
                key_s = 'cpu_times'
                if 'cpu_times_user' not in sorted_infos[node].keys():
                    temp = {'values': [0],'times':[0],'unit':''}
                else:
                    temp = sorted_infos[node]['cpu_times_user']
                if 'cpu_times_system' not in sorted_infos[node].keys():
                    temp1 = {'values': [0],'times':[0],'unit':''}
                else:
                    temp1 = sorted_infos[node]['cpu_times_system']
                plot_datas.append((temp['values'],temp['times'],temp['unit']))
                plot_datas.append((temp1['values'],temp1['times'],temp1['unit']))
                legend_ = ['cpu times user', 'cpu times system']
            elif j == 2:
                key_s = 'ram'
                if 'memory_info_rss' not in sorted_infos[node].keys():
                    temp = {'values': [0],'times':[0],'unit':''}
                else:
                    temp = sorted_infos[node]['memory_info_rss']
                if 'memory_info_vms' not in sorted_infos[node].keys():
                    temp1 = {'values': [0],'times':[0],'unit':''}
                else:
                    temp1 = sorted_infos[node]['memory_info_vms']

                plot_datas.append((temp['values'],temp['times'],temp['unit']))
                plot_datas.append((temp1['values'],temp1['times'],temp1['unit']))
                legend_ = ['physical ram', 'virtual ram']
            elif j == 3:
                key_s = 'ctx_switches'
                if 'num_ctx_switches_voluntary' not in sorted_infos[node].keys():
                    temp = {'values': [0],'times':[0],'unit':''}
                else:
                    temp = sorted_infos[node]['num_ctx_switches_voluntary']
                if 'num_ctx_switches_involuntary' not in sorted_infos[node].keys():
                    temp1 = {'values': [0],'times':[0],'unit':''}
                else:
                    temp1 = sorted_infos[node]['num_ctx_switches_involuntary']

                plot_datas.append((temp['values'],temp['times'],temp['unit']))
                plot_datas.append((temp1['values'],temp1['times'],temp1['unit']))
                legend_ = ['voluntary', 'involuntary']
            elif j == 4:
                key_s = 'io_count'
                if 'io_counters_read_count' not in sorted_infos[node].keys():
                    temp = {'values': [0],'times':[0],'unit':''}
                else:
                    temp = sorted_infos[node]['io_counters_read_count']
                if 'io_counters_write_count' not in sorted_infos[node].keys():
                    temp1 = {'values': [0],'times':[0],'unit':''}
                else:
                    temp1 = sorted_infos[node]['io_counters_write_count']

                plot_datas.append((temp['values'],temp['times'],temp['unit']))
                plot_datas.append((temp1['values'],temp1['times'],temp1['unit']))
                legend_ = ['read count', 'write count']
            elif j == 5:
                key_s = 'io_bytes'
                if 'io_counters_read_bytes' not in sorted_infos[node].keys():
                    temp = {'values': [0],'times':[0],'unit':''}
                else:
                    temp = sorted_infos[node]['io_counters_read_bytes']
                if 'io_counters_write_bytes' not in sorted_infos[node].keys():
                    temp1 = {'values': [0],'times':[0],'unit':''}
                else:
                    temp1 = sorted_infos[node]['io_counters_write_bytes']
                plot_datas.append((temp['values'],temp['times'],temp['unit']))
                plot_datas.append((temp1['values'],temp1['times'],temp1['unit']))
                legend_ = ['read bytes', 'write bytes']

            unit = ""
            mins = []
            maxs = []
            scl = 0
            for x, data_set in enumerate(plot_datas):
                mins.append(min(data_set[0]))
                maxs.append(max(data_set[0]))
                scl = scale[key_s]

                if len(data_set[0]) < 2:
                    ax[j].text(0.9, 0.5, 'No Messages',
                               verticalalignment='top', horizontalalignment='right',
                               transform=ax[j].transAxes,
                               color='blue', fontsize=10)
                else:
                    plt.plot(data_set[1], data_set[0], linewidth=1,label=legend_[x])
                unit = data_set[2]

                
            mi = min(mins)
            ma = max(maxs)
            scl2 = ma - mi

            if scl < 0:
                scl*=-1
            elif scl == 0:
                scl = 1
            if scl2 > scl:
                scl = scl2
            while scl < ma:
                scl = scl + scl*0.1
            l = [0.0, scl*0.25,scl*0.5,scl*0.75,scl]
            l.sort()

            plt.yticks(l)

            ax[j].legend(loc='best', fontsize=4.0)

            ax[j].xaxis.set_major_formatter(FormatStrFormatter('%.1f'))
            sf = ScalarFormatter(useMathText=True,useOffset = False)
            sf.set_scientific(True)
            sf.set_powerlimits((-3, 3))
            ax[j].yaxis.set_major_formatter(sf)
            plt.xlabel("time in sec",fontsize=4,labelpad=2)
            plt.tick_params(direction = 'in',pad = 2)
            plt.xticks(fontsize=3)
            #TODO title
            plt.title(key_s,fontsize=4)
            plt.subplots_adjust(left=0.05,bottom=0.05,top=0.9,right=0.95, wspace = 0.25, hspace = 0.4)
            plt.grid(True,linestyle=':',c='black')

            if not unit:
                plt.ylabel("")
            else:
                plt.ylabel(unit, fontsize=3,labelpad=2)
            plt.yticks(fontsize=4)

            #if intervention_times:
            #    for perc_t in intervention_times:
            #        plt.axvline(x=perc_t, ymin=0.0,
            #                    ymax = 0.99, linewidth=1, color='r')

            photo_name = node.replace('/','_')

        plt.savefig(photo_path+photo_name+s_name, bbox_inches='tight', dpi = 400)
        plt.close()
        i = i + 1

def plot(photo_path, blacklist, debug_mode_, inter_times):
    i = 1
    global legend
    global fig, ax
    global lined
    tabell_errors = {}

    #plt.style.use('ggplot')
    for argument in value_dict.keys():
        if argument not in tabell_errors.keys():
            tabell_errors[argument] = {}

        plt.figure(i)
        plt.figure(i).suptitle(argument)
        plt.tick_params(colors = 'black')
        j = 1
        size_of_plt = math.sqrt(len(value_dict[argument].keys()))
        #size_of_plt = math.sqrt(6)
        ab = []
        ax = []
        p = []
        for key in value_dict[argument].keys():
            if not value_dict[argument][key]['value'][0].replace('.','',1).isdigit():
                # value is not a number
                continue
            if key.replace(argument,'') in blacklist:
                continue

            ax.append(plt.subplot(size_of_plt, size_of_plt, j))
            if len(value_dict[argument][key]['value']) < 2:
                ax[j].text(0.9, 0.5, 'No Messages',
                               verticalalignment='top', horizontalalignment='right',
                               transform=ax[j - 1].transAxes,
                               color='blue', fontsize=10)

            if key not in tabell_errors[argument].keys():
                tabell_errors[argument][key] = value_dict[argument][key]['errorlevel']


            ax[j-1].yaxis.offsetText.set_fontsize(4)
            ax[j-1].yaxis.set_major_locator(plt.MaxNLocator(8))
            ax[j-1].xaxis.set_major_locator(plt.MaxNLocator(8))

            #data_list, unit = suit_unit(value_dict[argument][key]['value'], value_dict[argument][key]['unit'][0])
            data_list = value_dict[argument][key]['value']
            unit = value_dict[argument][key]['unit'][0]

            time_list = map(lambda x: float(x), value_dict[argument][key]['timestamp'])
            data_list = map(lambda x: float(x), data_list)

            plt.plot(time_list,data_list,'black',linewidth=1)
            ax[j-1].xaxis.set_major_formatter(FormatStrFormatter('%.1f'))
            sf = ScalarFormatter(useMathText=True,useOffset = False)
            sf.set_scientific(True)
            sf.set_powerlimits((-3, 3))
            ax[j-1].yaxis.set_major_formatter(sf)

            plt.xlabel("time in sec",fontsize=4,labelpad=2)
            plt.tick_params(direction = 'in',pad = 2)
            plt.xticks(fontsize=3)
            plt.title(key.replace(argument,''),fontsize=4)#,pad=2.0)
            plt.subplots_adjust(left=0.05,bottom=0.05,top=0.9,right=0.95, wspace = 0.25, hspace = 0.4)
            plt.grid(True,linestyle=':',c='black')

            if not unit:
                plt.ylabel("")
            else:
                plt.ylabel(unit, fontsize=3,labelpad=2)
            plt.yticks(fontsize=3)

            if inter_times:
                for perc_t in inter_times:
                    plt.axvline(x=perc_t, ymin=0.0, ymax = 0.99, linewidth=1, color='r')

            xy = [1.02, 1.02]
            da = DrawingArea(10, 10, 0, 0)
            p.append(Circle((3, 3), 4, color=value_dict[argument][key]['errorlevel']))

            da.add_artist(p[j-1])
            ab.append(copy(AnnotationBbox(da, xy, xybox=(0.98, 1.15),xycoords='figure fraction', boxcoords=("axes fraction"),frameon=False)))
            ax[j-1].add_artist(ab[j-1])

            if debug_mode_ == True:
                for criterion_name in value_dict[argument][key]['criterion_value'].keys():
                    debug_msg = criterion_name + ":\n" + str(value_dict[argument][key]['criterion_value'][criterion_name])
                    ax[j-1].text(0.3, 1.3,debug_msg,
                            verticalalignment='top', horizontalalignment='right',
                            transform=ax[j-1].transAxes,
                            color='blue', fontsize=4)
            j = j+1
            photo_name = argument.replace('/','_')

        plt.savefig(photo_path+photo_name, bbox_inches='tight', dpi = 400)
        plt.close()
        i = i + 1
    safe_yaml_to_file(tabell_errors, photo_path)

def safe_yaml_to_file(dict_, photo_path):
    f1 =  yaml.dump(dict_, default_flow_style=False)
    try:
        f = open(photo_path+'/infofile.yaml','w')   
        f.write(f1)
        f.close()
    except Exception as inst:
        print(str(inst))

def suit_unit(data_list, unit):
    byte_unit = ["byte", "kB", "MB", "GB"]
    time_unit = ["ms", "sec","h"]
    unit_now = unit
    value_list = data_list
    if unit_now in "bytes":
        unit_now = "byte"
    if unit_now in byte_unit:
        while min(value_list) > 1024:
            value_list = map(lambda x:float(x)/1024,value_list)
            unit_now = byte_unit[byte_unit.index(unit_now)+1]
    return value_list, unit_now




if __name__ == '__main__':
    init()
