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

#from cycler import cycler
import matplotlib.pyplot as plt
import matplotlib.axis as ax
from matplotlib.patches import Circle
from matplotlib.offsetbox import (TextArea, DrawingArea, AnnotationBbox)
from matplotlib.font_manager import FontProperties
from matplotlib.ticker import FormatStrFormatter, ScalarFormatter
from matplotlib.rcsetup import cycler

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

def get_bag_data(bag_dir):
    temp_list = []
    other_topics = []
    d_inter = []
    bag = rosbag.Bag(bag_dir)
    for topic, msg, t in bag.read_messages():
        # t is rospy time: t.secs  t.nsecs
        if topic == "/monitoring":
            temp_list.append((msg, t))
        if topic == '/E_VEHICLE/id0/E10_driver_intervention':
            d_inter.append((msg, t))
        if topic not in ['/diagnostics', '/rosout_agg',
                '/statistics', 'tf', 'tf_static']:
            other_topics.append((topic, msg, t))
    return temp_list, d_inter, other_topics

def get_infos(data, inter_stamps):
    ping_dict, cpu_dict, freq_dict, ram_dict, net_dict = {}, {}, {}, {}, {}
    node_dict, ntp_dict = {}, {}
    start_time = None
    times = []
    hosts = []
    for msg, time in data:
        if not start_time:
            start_time = time
        for mon_info in msg.info:
            temp = mon_info.name.split('/')[0]
            if temp not in hosts:
                hosts.append(temp)
            if "ping" in mon_info.name:
                host = mon_info.name.split('/')[0]
                if host not in ping_dict.keys():
                    ping_dict[host] = {}
                for value in mon_info.values:
                    ip = value.key.split('/')[0]
                    if ip not in ping_dict[host].keys():
                        ping_dict[host][ip] = {'delays':[], 'times':[]}
                    delay = None
                    try:
                        delay = float(value.value)
                    except Exception as e:
                        pass
                    if not delay:
                        delay = -10.0
                    ping_dict[host][ip]['delays'].append(delay)
                    ping_dict[host][ip]['times'].append((time-start_time).to_sec())
            elif "frequency" in mon_info.name:
                host = mon_info.name.split('/')[0]
                if host not in freq_dict.keys():
                    freq_dict[host] = {}

                for value in mon_info.values:
                    core = value.key.split('/')[0]
                    if core not in freq_dict[host].keys():
                        freq_dict[host][core] = {'freq':[], 'times':[], 'max_freq':1.0}

                    if "frequency" in value.key:
                        freq_dict[host][core]['freq'].append(float(value.value))
                        freq_dict[host][core]['times'].append((time-start_time).to_sec())
                    if "max_freq" in value.key:
                        #print "Max Freq: " +str(value.value)
                        freq_dict[host][core]['max_freq'] = float(value.value)
            elif "cpu" in mon_info.name and "frequency" not in mon_info.name:
                host = mon_info.name.split('/')[0]
                if host not in cpu_dict.keys():
                    cpu_dict[host] = {'load': [], 'temp':[], 'times': []}

                cpu_dict[host]['times'].append((time-start_time).to_sec())
                for value in mon_info.values:
                    if "load" == value.key:
                        cpu_dict[host]['load'].append(float(value.value))
                    elif "temp" == value.key:
                        cpu_dict[host]['temp'].append(float(value.value))
            elif "ram_monitor" in mon_info.name:
                host = mon_info.name.split('/')[0]

                if host not in ram_dict.keys():
                    ram_dict[host] = {'usage': [], 'times': []}
                ram_dict[host]['times'].append((time-start_time).to_sec())
                for value in mon_info.values:
                    if "percentage_used" == value.key:
                        ram_dict[host]['usage'].append(float(value.value))
            elif 'network' in mon_info.name:
                host = mon_info.name.split('/')[0]
                network = mon_info.name.split('/')[-1].split('_')[-1]
                if host not in net_dict.keys():
                    net_dict[host] = {}
                if network not in net_dict[host].keys():
                    net_dict[host][network] = {'RX': [], 'TX':[], 'load_RX': [],
                        'load_TX':[], 'times':[]}
                net_dict[host][network]['times'].append((time-start_time).to_sec())
                for value in mon_info.values:
                    if "RX" == value.key:
                        net_dict[host][network]['RX'].append(
                            float(value.value))
                    elif "TX" == value.key:
                        net_dict[host][network]['TX'].append(
                            float(value.value))
                    elif "load_RX" == value.key:
                        net_dict[host][network]['load_RX'].append(
                            float(value.value))
                    if "load_TX" == value.key:
                        net_dict[host][network]['load_TX'].append(
                            float(value.value))
            elif 'ntp' in mon_info.name:
                host = mon_info.name.split('/')[0]

                if host not in ntp_dict.keys():
                    ntp_dict[host] = {}

                for value in mon_info.values:
                    ip = value.key.split('/')[0]
                    key = value.key.split('/')[-1]

                    if ip not in ntp_dict[host].keys():
                        ntp_dict[host][ip] = {'delay': [], 'offset':[], 'times':[]}

                    if "ntp_offset" in key:
                        ntp_dict[host][ip]['offset'].append(
                            float(value.value))
                    if "delay" in key:
                        #print ip + " " + key + " " + str(value.value)
                        ntp_dict[host][ip]['delay'].append(
                            float(value.value))
                    if "ntp_time" == key:
                        ntp_dict[host][ip]['times'].append((time-start_time).to_sec())
            elif 'node_monitor' in mon_info.name:
                host = mon_info.name.split('/')[0]
                if host not in node_dict.keys():
                    node_dict[host] = {}
                for val in mon_info.values:
                    node_name = val.key
                    if node_name not in node_dict[host].keys():
                        node_dict[host][node_name] = {'unavailable': []}
                    node_dict[host][node_name]['unavailable'].append((time-start_time).to_sec())
            else:
                pass

    for msg,t in inter_stamps:
        temp = (t-start_time).to_sec()
        times.append(int(temp))
    return ping_dict, cpu_dict, freq_dict, ram_dict, net_dict, node_dict, ntp_dict, times, hosts

def sort_data(ping_dict, cpu_dict, freq_dict, ram_dict, net_dict, node_dict, ntp_dict, hosts):
    val_dict = {}
    for host in ping_dict.keys():
        if host not in val_dict.keys():
            val_dict[host] = {}
        val_dict[host]['ping'] = ping_dict[host]

    for host in cpu_dict.keys():
        if host not in val_dict.keys():
            val_dict[host] = {}
        val_dict[host]['cpu_load'] = cpu_dict[host]

    for host in freq_dict.keys():
        if host not in val_dict.keys():
            val_dict[host] = {}
        val_dict[host]['cpu_freq'] = freq_dict[host]

    for host in ram_dict.keys():
        if host not in val_dict.keys():
            val_dict[host] = {}
        val_dict[host]['ram_usage'] = ram_dict[host]

    for host in net_dict.keys():
        if host not in val_dict.keys():
            val_dict[host] = {}
        val_dict[host]['network'] = net_dict[host]

    for host in node_dict.keys():
        if host not in val_dict.keys():
            val_dict[host] = {}
        val_dict[host]['node_unaviable'] = node_dict[host]

    for host in ntp_dict.keys():
        if host not in val_dict.keys():
            val_dict[host] = {}
        val_dict[host]['ntp_info'] = ntp_dict[host]

    return val_dict

        #fig, subplots = plot.subplots(x, y)


def plot(val_dict, intervention_times, spath):
    try:
        if not os.path.exists(spath):
            os.makedirs(spath)
    except Exception as e:
        print e
        return
    i = 0
    size_of_plt, num_plt = None, 0
    cm = plt.get_cmap('gist_rainbow')
    colors = [cm(i) for i in numpy.linspace(0, 1, 16)]
    color_cycler = cycler(color=colors)
    #color_cycler = cycler(color=colors)
    
    for host in val_dict.keys():

        num_plt = len(val_dict[host])
        size_of_plt = int(math.sqrt(num_plt)+0.99)
        plt.figure(i)
        plt.figure(i).suptitle(host)
        plt.tick_params(colors = 'black')
        ab, ax, p = [], [], []
        print "Host: "+host + " Max Plots: "+str(len(val_dict[host].keys()))
        for j, key in enumerate(val_dict[host].keys()):
            #create the plots for ntp, cpu, freq, etc.
            print "Host: "+ host + " Key: "+key
            ax.append(plt.subplot(size_of_plt,size_of_plt,j+1))
            #subplots[0, 0].set_prop_cycle(color_cycler)
            #subplots[1, 0].set_prop_cycle(color_cycler)
            #subplots[0, 1].set_prop_cycle(color_cycler)
            ax[j].yaxis.offsetText.set_fontsize(4)
            ax[j].yaxis.set_major_locator(plt.MaxNLocator(8))
            ax[j].xaxis.set_major_locator(plt.MaxNLocator(8))
            #ax[j].set_prop_cycle(color_cycler)

            time_list = []
            data_list = []
            legend = []
            unit = ''
            title = key

            data = val_dict[host][key]
            if key == 'ping':
                unit = 'msec'
                #ping_dict[host][ip] = {'delays':[], 'times':[]}
                for ip in data.keys():
                    legend.append(ip)
                    data_list.append(data[ip]['delays'])
                    time_list.append(data[ip]['times'])
            elif key == 'cpu_load':
                unit='Temp / %'
                #cpu_dict[host] = {'load': [], 'temp':[], 'times': []}
                data_list.append(data['load'])
                data_list.append(data['temp'])
                legend.append('Load')
                legend.append('Temperatur')
                time_list.append(data['times'])
                time_list.append(data['times'])
            elif key == 'cpu_freq':
                #freq_dict[host][core] = {'freq':[], 'times':[], 'max_freq':''}
                unit = 'percent'
                for core in data.keys():
                    #legend.append(core)
                    maxf = data[core]['max_freq']
                    #print maxf
                    temp = [x/maxf for x in data[core]['freq']]
                    data_list.append(temp)
                    legend.append(core)
                    time_list.append(data[core]['times'])
            elif key == 'ram_usage':
                unit='percent'
                data_list.append(data['usage'])
                legend.append("ram [%]")
                time_list.append(data['times'])
            elif key == 'network':
                #net_dict[host][network] = {'RX': [], 'TX':[], 'load_RX': [],
                #        'load_TX':[], 'times':[]}
                unit = ''
                for network in data.keys():
                    data_list.append(data[network]['RX'])
                    data_list.append(data[network]['TX'])
                    data_list.append(data[network]['load_RX'])
                    data_list.append(data[network]['load_TX'])
                    legend.append(network+"_RX-count")
                    legend.append(network+"_TX-count")
                    legend.append(network+"_RX-load")
                    legend.append(network+"_TX-load")
                    time_list.append(data[network]['times'])
                    time_list.append(data[network]['times'])
                    time_list.append(data[network]['times'])
                    time_list.append(data[network]['times'])
            elif key == 'node_unaviable':
                unit = ''
                #node_dict[host][node_name] = {'unavailable': []}
                for node in data.keys():
                    legend.append(node)
                    temp = data[node]['unavailable']
                    data_list.append([1]*len(temp))
                    time_list.append(temp)
            elif key == 'ntp_info':
                unit = 'sec'
                #ntp_dict[host][ip] = {'delay': [], 'offset':[], 'times':[]}
                for ip in data.keys():
                    legend.append(ip+"_delay")
                    temp = data[ip]['delay']
                    #print temp
                    #temp[:] = [-10 for i in temp if i >= 1000]
                    for x,val in enumerate(temp):
                        if val >= 1000.0:
                            temp[x] = -10
                    data_list.append(temp)
                    legend.append(ip+"_offset")
                    temp2 = data[ip]['offset']
                    #temp2[:] = [-10 for i in temp2 if i >= 1000]
                    for x,val in enumerate(temp2):
                        if val >= 1000.0:
                            temp2[x] = -10
                    data_list.append(temp)
                    time_list.append(data[ip]['times'])
                    time_list.append(data[ip]['times'])


            ax2 = None
            for k,p in enumerate(data_list):
                if len(time_list[k]) != len(p):
                    print "Wrong Len For Key: "+key+" Host: "+host
                else:
                    if key == 'node_unaviable':
                        plt.plot(time_list[k], p, marker='.', markersize=0.01,
                            label=legend[k])
                        ax[j].legend(loc='upper right', fontsize=4.0)
                        #ax[j].legend()
                    elif key == 'cpu_load':
                        plt.plot(time_list[k], p, label=legend[k])
                        ax[j].legend(loc='best', fontsize=4.0)
                    elif key == 'ping':
                        plt.plot(time_list[k], p, label=legend[k])
                        ax[j].legend(loc='best', fontsize=4.0)
                    elif key == 'network':
                        leg = legend[k]

                        if 'count' in leg:
                            ax[j].plot(time_list[k], p, label=leg)
                            #ax[j].legend(fontsize=4.0, loc=0)
                            plt.yticks(fontsize=3)
                            plt.xticks(fontsize=3)
                            #ax[j].legend(loc=0, fontsize=4.0)
                        elif 'load' in leg:
                            if not ax2:
                                ax2 = ax[j].twinx()
                                ax2.set_prop_cycle(color_cycler)
                                ax2.yaxis.offsetText.set_fontsize(4)
                                ax2.yaxis.set_major_locator(plt.MaxNLocator(8))
                                ax2.xaxis.set_major_locator(plt.MaxNLocator(8))
                            temp = [a*1000000000 for a in p]
                            ax2.plot(time_list[k], temp, marker='.', label=leg)
                            #ax2.legend(loc=0, fontsize=4.0)
                            plt.yticks(fontsize=3)
                            plt.xticks(fontsize=3)
                        #ax[j].legend(fontsize=3.0, loc='best')
                        #if ax2:
                        #    ax2.legend(loc=0, fontsize=3.0)
                    elif key == 'ntp_info':
                        #TODO make it nicer
                        plt.plot(time_list[k], p, label=legend[k])
                        ax[j].legend(loc='best', fontsize=4.0)
                    elif key == 'cpu_freq':
                        plt.plot(time_list[k], p, label=legend[k])
                        ax[j].legend(loc='best', fontsize=4.0)
                    elif key == 'ram_usage':
                        plt.plot(time_list[k], p, label=legend[k])
                        plt.yticks([0,10,20,30,40,50,60,70,80,90,100])
                        ax[j].legend(loc='best', fontsize=4.0)
                    else:
                        plt.plot(time_list[k], p)
            if key == 'network':
                lines, labels = ax[j].get_legend_handles_labels()
                lines2, labels2 = ax2.get_legend_handles_labels()
                ax2.legend(lines + lines2, labels + labels2, loc=0, fontsize=3.0)
                ax2.set_ylabel("Load in Bit/s", fontsize=3)
                ax[j].set_ylabel("Count", fontsize=3)


            #plt.plot(time_list,data_list, 'black', linestyle='-', linewidth=1)

            #plt.plot(time_list, data_list, marker='o', linestyle='--', color='black')

            #if intervention_times:
            #    for perc_t in intervention_times:
            #        plt.axvline(x=perc_t, ymin=0.0, ymax = 0.99, linewidth=1, color='r')

            ax[j].xaxis.set_major_formatter(FormatStrFormatter('%.1f'))
            sf = ScalarFormatter(useMathText=True, useOffset = False)
            sf.set_scientific(True)
            sf.set_powerlimits((-3, 3))
            ax[j].yaxis.set_major_formatter(sf)
            if ax2:
                ax2.xaxis.set_major_formatter(FormatStrFormatter('%.1f'))
                ax2.yaxis.set_major_formatter(sf)
            plt.xlabel("time in sec", fontsize = 4, labelpad = 2)
            plt.tick_params(direction = 'in',pad = 2)
            plt.xticks(fontsize = 3)
            plt.title(title, fontsize=4)#,pad=2.0)
            plt.subplots_adjust(left=0.05,bottom=0.05,top=0.9,right=0.95, wspace = 0.25, hspace = 0.4)
            plt.grid(True,linestyle=':',c='black')
            if unit:
                plt.ylabel(unit, fontsize=3,labelpad=2)
            plt.yticks(fontsize=3)
        name = host
        name = name.replace('/','_')
        plt.savefig(spath+name, bbox_inches='tight', dpi = 400)
        plt.close()

        i+=1


def get_time_diffs(data, inter_data):
    times = []
    inter_times = []
    mon_bag_t_diff = []
    mon_arr_info_diffs = []
    for monitoring_arr in data:
        mon_arr_now = monitoring_arr[0]
        bag_time = monitoring_arr[1]
        times.append(bag_time.secs)
        mon_bag_t_diff.append((bag_time - mon_arr_now.header.stamp).to_sec())
        temp_list = []
        for mon_info in mon_arr_now.info:
            temp_list.append((mon_arr_now.header.stamp - mon_info.header.stamp).to_sec())
        mon_arr_info_diffs.append(temp_list)
    return times, mon_arr_info_diffs, mon_bag_t_diff, inter_times


def main():
    bpath, spath = parse_sys_args()
    if not bpath or not spath:
        return
    data, intervention_data, other_data = get_bag_data(bpath)
    ping_dict, cpu_dict, freq_dict, ram_dict, net_dict, node_dict, ntp_dict, times, hosts = get_infos(data, intervention_data)
    val_dict = sort_data(ping_dict, cpu_dict, freq_dict, ram_dict, net_dict, node_dict, ntp_dict, hosts)
    plot(val_dict, times, spath)

    #times, header_diffs, header_rb_diff, inter_times = get_time_diffs(data, intervention_data)
    #indexs = range(len(header_diffs))
    #plt.figure()
    #for i, elem in enumerate(header_diffs):
    #    for val in elem:
    #        print val
    #        plt.plot(i, val)
    #plt.show()


if __name__=='__main__':
    main()
