#!/usr/bin/env python
import rospy
import sys
import os
import yaml
import rosbag
import math
import numpy
from copy import copy
from collections import OrderedDict
from rosgraph_msgs.msg import TopicStatistics

import matplotlib.pyplot as plt
import matplotlib.axis as ax
from matplotlib.patches import Circle
from matplotlib.offsetbox import (TextArea, DrawingArea, AnnotationBbox)
from matplotlib.font_manager import FontProperties
from matplotlib.ticker import FormatStrFormatter, ScalarFormatter

def parse_sys_args():
    if len(sys.argv) > 1:
        path = sys.argv[1]
        spath = sys.argv[2]
        return path, spath
    else:
        print "required 2 args"
        print "1 Arg:   /Path/for/loading/bagfile.bag"
        print "2 Arg: path/to/safe/pictures.png"
        return None, None

def get_bag_data(bag_dir):
    temp_list = []
    start_time = None
    bag = rosbag.Bag(bag_dir)
    for topic, msg, t in bag.read_messages():
        if not start_time:
            start_time = t.to_sec()
        if t.to_sec() - start_time < 0.0:
            start_time = t.to_sec()
        # t is rospy time: t.secs  t.nsecs
        if topic == "/monitoring":
            for info in msg.info:
                if info.description == "statistics_monitor":
                    #if info.
                    #print info
                    time = info.header.stamp.to_sec()
                    if time < start_time:
                        start_time = time
                    vals = info.values
                    temp_list.append((vals, time))
    return temp_list, start_time

def sort_data(stat_list, start_time):
    val_dict = {}

    for element in stat_list:
        vals = element[0]
        time = element[1]
        ctime = time - start_time
        for val in vals:
            l = [pos for pos, c in enumerate(val.key) if c == "/"][-1]
            s = val.key.find("/")
            topic = val.key[:s]
            pub_sub = val.key[s+1:l]
            key2 = val.key[l+1:]
            #print val.key
            #print topic
            #print pub_sub
            #print key2
            #print "\n"

            if not pub_sub:
                continue
            if topic not in val_dict.keys():
                val_dict[topic] = {}
            if pub_sub not in val_dict[topic].keys():
                val_dict[topic][pub_sub] = {'timestamp':[], 'Byte_per_Msg':([],[]),
                    'bandwidth':([],[]), 'Delivered':([],[]), 'Dropped':([],[]),
                    'StampAge':([],[]),'Frequenzy':([],[])}
            if ctime not in val_dict[topic][pub_sub]['timestamp']:
                val_dict[topic][pub_sub]['timestamp'].append(ctime)

            if key2 == "current_msg_delivered":
                val_dict[topic][pub_sub]['Delivered'][0].append(float(val.value))
                val_dict[topic][pub_sub]['Delivered'][1].append(ctime)
            elif key2 == "current_msg_dropped":
                val_dict[topic][pub_sub]['Dropped'][0].append(float(val.value))
                val_dict[topic][pub_sub]['Dropped'][1].append(ctime)
            elif key2 == "frequency":
                val_dict[topic][pub_sub]['Frequenzy'][0].append(float(val.value))
                val_dict[topic][pub_sub]['Frequenzy'][1].append(ctime)
            elif key2 == "stamp_age":
                val_dict[topic][pub_sub]['StampAge'][0].append(float(val.value))
                val_dict[topic][pub_sub]['StampAge'][1].append(ctime)
            elif key2 == "avg_bandwidth":
                val_dict[topic][pub_sub]['bandwidth'][0].append(float(val.value))
                val_dict[topic][pub_sub]['bandwidth'][1].append(ctime)
            elif key2 == "byte_per_msg":
                val_dict[topic][pub_sub]['Byte_per_Msg'][0].append(float(val.value))
                val_dict[topic][pub_sub]['Byte_per_Msg'][1].append(ctime)
    #for topic in val_dict.keys():
    #    for connection in val_dict[topic].keys():
    #        for key in val_dict[topic][connection].keys():
    #            if len(val_dict[topic][connection][key]) < 1:
    #                print topic + ":::"+ connection
    #                print "\n"
    #    print "\n\n"
    return val_dict

def plot(val_dict, spath):
    try:
        if not os.path.exists(spath):
            os.makedirs(spath)
    except Exception as e:
        print e
        return
    i = 0
    size_of_plt, num_plt = None, 0
    for topic in val_dict.keys():
        for connection in val_dict[topic].keys():
            if not size_of_plt:
                num_plt = len(val_dict[topic][connection]) - 1
                size_of_plt = int(math.sqrt(num_plt)+0.99)
            plt.figure(i)
            plt.figure(i).suptitle(topic+": "+connection)
            plt.tick_params(colors = 'black')
            ab, ax, p = [], [], []
            for j in range(num_plt):
                #create the plots for bw, freq, StampAge, etc.
                ax.append(plt.subplot(size_of_plt,size_of_plt,j+1))
                if len(val_dict[topic][connection]['timestamp']) < 2:
                    # only one msg, to less to plot
                    ax[j].text(0.9, 0.5, 'No Messages',
                               verticalalignment='top',
                               horizontalalignment='right',
                               transform=ax[j].transAxes,
                               color='blue', fontsize=10)

                ax[j].yaxis.offsetText.set_fontsize(4)
                ax[j].yaxis.set_major_locator(plt.MaxNLocator(8))
                ax[j].xaxis.set_major_locator(plt.MaxNLocator(8))
                #time_list = val_dict[topic][connection]['timestamp']
                data_list, time_list, unit, title, yscale = get_data_unit(val_dict, topic, connection, j)

                if data_list and len(time_list) == len(data_list):
                    if len(val_dict[topic][connection]['timestamp']) > 10:
                        # 10 is arbitary need to be adjusted depending on time
                        plt.plot(time_list,data_list, 'black', linestyle='-', linewidth=1)
                    else:
                        plt.plot(time_list, data_list, marker='o', linestyle='--', color='black')

                    ax[j].xaxis.set_major_formatter(FormatStrFormatter('%.1f'))
                    sf = ScalarFormatter(useMathText=True, useOffset = False)
                    sf.set_scientific(True)
                    sf.set_powerlimits((-3, 3))
                    ax[j].yaxis.set_major_formatter(sf)
                    plt.xlabel("time in sec", fontsize = 4, labelpad = 2)
                    plt.tick_params(direction = 'in',pad = 2)
                    plt.xticks(fontsize = 3)
                    plt.title(title, fontsize=4)#,pad=2.0)
                    plt.subplots_adjust(left=0.05,bottom=0.05,top=0.9,right=0.95, wspace = 0.25, hspace = 0.4)
                    plt.grid(True,linestyle=':',c='black')
                    if not unit:
                        plt.ylabel("")
                    else:
                        plt.ylabel(unit, fontsize=3,labelpad=2)

                    plt.yticks(yscale, fontsize=3)

            name = topic+":"+connection
            name = name.replace('/','_')
            plt.savefig(spath+name, bbox_inches='tight', dpi = 400)
            plt.close()
            i+=1

def plot_per_topic(val_dict, intervention_times, spath):
    try:
        if not os.path.exists(spath):
            os.makedirs(spath)
    except Exception as e:
        print e
        return
    i = 0
    size_of_plt, num_plt = None, 0
    for topic in val_dict.keys():
        plt.figure(i)
        plt.figure(i).suptitle(topic)
        plt.tick_params(colors = 'black')
        ab, ax, p = [], [], []
        setup = False
        if not size_of_plt:
            num_plt = len(val_dict[topic][val_dict[topic].keys()[0]]) - 1
            size_of_plt = int(math.sqrt(num_plt)+0.99)

        for j in range(num_plt):
            ax.append(plt.subplot(size_of_plt,size_of_plt,j+1))
            ax[j].yaxis.offsetText.set_fontsize(4)
            ax[j].yaxis.set_major_locator(plt.MaxNLocator(8))
            ax[j].xaxis.set_major_locator(plt.MaxNLocator(8))

            for connection in val_dict[topic].keys():
                time_list = val_dict[topic][connection]['timestamp']
                data_list, time_list, unit, title, yscale = get_data_unit(val_dict, topic, connection, j)

                if len(val_dict[topic][connection]['timestamp']) > 10:
                    # 10 is arbitary need to be adjusted depending on time
                    plt.plot(time_list,data_list, linestyle='-',
                             linewidth=1, label=connection)
                else:
                    plt.plot(time_list, data_list, marker='o', linestyle='--', label=connection)
            ax[j].legend(loc='best', fontsize=4.0)
            ax[j].xaxis.set_major_formatter(FormatStrFormatter('%.1f'))
            sf = ScalarFormatter(useMathText=True, useOffset = False)
            sf.set_scientific(True)
            sf.set_powerlimits((-3, 3))
            ax[j].yaxis.set_major_formatter(sf)
            plt.xlabel("time in sec", fontsize = 4, labelpad = 2)
            plt.tick_params(direction = 'in',pad = 2)
            plt.xticks(fontsize = 3)
            plt.title(title, fontsize=4)#,pad=2.0)
            plt.subplots_adjust(left=0.05,bottom=0.05,top=0.9,right=0.95, wspace = 0.25, hspace = 0.4)
            plt.grid(True,linestyle=':',c='black')
            if not unit:
                plt.ylabel("")
            else:
                plt.ylabel(unit, fontsize=3,labelpad=2)

            plt.yticks(fontsize=3)
        #for loop end
        name = topic
        name = name.replace('/','_')
        plt.savefig(spath+'topic-'+name, bbox_inches='tight', dpi = 400)
        plt.close()
        i+=1

def get_data_unit(val_dict, topic, connection, j):
    if j == 0:
        data_list = val_dict[topic][connection]['Byte_per_Msg'][0]
        times = val_dict[topic][connection]['Byte_per_Msg'][1]
        title = 'Byte_per_Msg'
        unit = 'Byte/Msg'
    elif j == 1:
        data_list = val_dict[topic][connection]['Delivered'][0]
        times = val_dict[topic][connection]['Delivered'][1]
        title = 'Delivered'
        unit = ''
    elif j == 2:
        data_list = val_dict[topic][connection]['Dropped'][0]
        times = val_dict[topic][connection]['Dropped'][1]
        title = 'Dropped'
        unit = ''
    #elif j == 3:
    #    data_list = val_dict[topic][connection]['Traffic']
    #    title = 'Traffic'
    #    unit = 'KBit'
    elif j == 3:
        data_list = val_dict[topic][connection]['StampAge'][0]
        times = val_dict[topic][connection]['StampAge'][1]
        title = 'StampAge'
        unit = 's'
    elif j == 4:
        data_list = val_dict[topic][connection]['Frequenzy'][0]
        times = val_dict[topic][connection]['Frequenzy'][1]
        title = 'Frequenzy'
        unit = 'Hz'
    elif j == 5:
        data_list = val_dict[topic][connection]['bandwidth'][0]
        times = val_dict[topic][connection]['bandwidth'][1]
        title = 'bandwidth'
        unit = 'KBit/s'
    if not data_list:
        print topic
        print connection
        print title
        print "\n"
        scale = [0,1,2,3]
    else:
        vmin, vmax = min(data_list), max(data_list)
        span = vmax - vmin
        scale = [vmin-span, vmin-span/2, vmin, vmin+span/2, vmax, vmax+span/2, vmax+span]
    #if span < 0.1:
    #    scale = [vmin-span, vmin, vmax, vmax+span]
    #    return data_list, unit, title, scale

    #if (vmin - span) < 0.0:
    #    vmin = 0.0
    #    span = span *3
    #    vmax = span
    #else:
    #    vmin = vmin - span
    #    span = span *3
    #    vmax = vmin + span


    #count = 0.0
    #dx = span / 9.0
    #scale = []
    #while vmin < vmax:
    #    scale.append(round(vmin,4))
    #    vmin += dx
    return data_list, times, unit, title, scale


def main():
    bpath, spath = parse_sys_args()
    if not bpath or not spath:
        return
    stat_data, start_time = get_bag_data(bpath)
    value_dict = sort_data(stat_data, start_time)
    plot(value_dict, spath)
    #plot_per_topic(value_dict, intervention_times, spath)



if __name__=='__main__':
    main()
