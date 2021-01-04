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
        mode = 0
        path = sys.argv[1]
        spath = sys.argv[2]
        try:
            mode = int(sys.argv[3])
        except Exception as e:
            pass
        finally:
            mode = 0
        if not(type(mode)==int):
            mode = 0
            print("3rd arg have to be an int type")
        if not ((mode == 0) or (mode == 1)):
            mode = 0
            print("mode had wrong value --> 0 = normal / 1 = average")
        return path, spath, mode
    else:
        print "required 2 args + 1 optional arg"
        print "1 Arg:   /Path/for/loading/bagfile.bag"
        print "2 Arg: path/to/safe/pictures.png"
        print "3 Arg: Modus(0 = normal plot / 1 = plot averages)"
        return None, None, None

def get_bag_data(bag_dir):
    temp_list = []
    d_inter = []
    bag = rosbag.Bag(bag_dir)
    for topic, msg, t in bag.read_messages():
        # t is rospy time: t.secs  t.nsecs
        if topic == "/statistics":
            temp_list.append((msg, t))
        if topic == '/E_VEHICLE/id0/E10_driver_intervention':
            d_inter.append(t)
    return temp_list, d_inter

def sort_data(stat_list, inter_stamps):
    start_time = None
    last_time = None
    val_dict = {}
    times = []
    for element in stat_list:
        msg = element[0]
        time = element[1]
        if not start_time:
            start_time = time
        # Get the interesting values
        name = msg.topic
        top_sub = msg.node_pub+"--->"+msg.node_sub
        byte_per_msg = (float(msg.traffic)) / (msg.delivered_msgs)
        bw = (msg.traffic*8/1000) / (msg.window_stop - msg.window_start).to_sec()
        delivered = msg.delivered_msgs
        traffic = msg.traffic * 8/1000
        dropped = msg.dropped_msgs
        stamp_mean_age = msg.stamp_age_mean.to_sec()
        if msg.period_mean.to_sec():
            freq = 1 / msg.period_mean.to_sec()
        else:
            freq = 0
        #Create dictionary entry if not existent
        if name not in val_dict.keys():
            val_dict[name] = {}
        if top_sub not in val_dict[name].keys():
            val_dict[name][top_sub] = {'timestamp':[], 'Byte_per_Msg':[],
                'bandwidth':[], 'Delivered':[], 'Dropped':[], #'Traffic':[],
                'StampAge':[],'Frequenzy':[]}
        # safe values in dictionary
        t = (time - start_time).to_sec()
        val_dict[name][top_sub]['timestamp'].append(t)
        val_dict[name][top_sub]['Byte_per_Msg'].append(byte_per_msg)
        val_dict[name][top_sub]['Delivered'].append(delivered)
        val_dict[name][top_sub]['Dropped'].append(dropped)
        #val_dict[name][top_sub]['Traffic'].append(traffic)
        val_dict[name][top_sub]['StampAge'].append(stamp_mean_age)
        val_dict[name][top_sub]['Frequenzy'].append(freq)
        val_dict[name][top_sub]['bandwidth'].append(bw)
        last_time = time

    #intervall = (last_time - start_time).to_sec()
    for t in inter_stamps:
        temp = (t-start_time).to_sec()
        times.append(int(temp))

    return val_dict, times

def plot(val_dict, intervention_times, spath):
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
                time_list = val_dict[topic][connection]['timestamp']
                data_list, unit, title, yscale = get_data_unit(val_dict, topic, connection, j)

                if len(val_dict[topic][connection]['timestamp']) > 10:
                    # 10 is arbitary need to be adjusted depending on time
                    plt.plot(time_list,data_list, 'black', linestyle='-', linewidth=1)
                else:
                    plt.plot(time_list, data_list, marker='o', linestyle='--', color='black')
                if intervention_times:
                    for perc_t in intervention_times:
                        plt.axvline(x=perc_t, ymin=0.0, ymax = 0.99, linewidth=1, color='r')

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
                data_list, unit, title, yscale = get_data_unit(val_dict, topic, connection, j)

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
        data_list = val_dict[topic][connection]['Byte_per_Msg']
        title = 'Byte_per_Msg'
        unit = 'Byte/Msg'
    elif j == 1:
        data_list = val_dict[topic][connection]['Delivered']
        title = 'Delivered'
        unit = ''
    elif j == 2:
        data_list = val_dict[topic][connection]['Dropped']
        title = 'Dropped'
        unit = ''
    #elif j == 3:
    #    data_list = val_dict[topic][connection]['Traffic']
    #    title = 'Traffic'
    #    unit = 'KBit'
    elif j == 3:
        data_list = val_dict[topic][connection]['StampAge']
        title = 'StampAge'
        unit = 's'
    elif j == 4:
        data_list = val_dict[topic][connection]['Frequenzy']
        title = 'Frequenzy'
        unit = 'Hz'
    elif j == 5:
        data_list = val_dict[topic][connection]['bandwidth']
        title = 'bandwidth'
        unit = 'KBit/s'
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
    return data_list, unit, title, scale


def main():
    bpath, spath, mode = parse_sys_args()
    if not bpath or not spath:
        return
    stat_data, intervention_stamps = get_bag_data(bpath)
    value_dict, intervention_times = sort_data(stat_data, intervention_stamps)
    if mode == 0:
        plot(value_dict, intervention_times, spath)
    elif mode == 1:
        plot_per_topic(value_dict, intervention_times, spath)

    #plot_avg_per_topic(value_dict, intervention_times, spath)


if __name__=='__main__':
    main()
