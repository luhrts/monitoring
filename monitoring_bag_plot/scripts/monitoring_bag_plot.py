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


def init():
    rospy.init_node("monitoring_bag_plot")
    if(rospy.has_param(rospy.get_name()+'/bag_dir')):
        bag_dir = rospy.get_param(rospy.get_name()+'/bag_dir')
    else:
        print 'no bag_dir supply'
        bag_dir = ''
    if(rospy.has_param(rospy.get_name()+'/monitor_type')):
        monitor_type = rospy.get_param(rospy.get_name()+'/monitor_type')
    else:
        print 'no monitor_type supply'
        monitor_type = ''
    if(rospy.has_param(rospy.get_name()+'/photo_path')):
        photo_path = rospy.get_param(rospy.get_name()+'/photo_path')
    else:
        print 'no photo_path supply'
        photo_path = ''
    if(rospy.has_param(rospy.get_name()+'/blacklist')):
        blacklist = rospy.get_param(rospy.get_name()+'/blacklist')
    else:
        print 'no blacklist supply'
        blacklist = ''
    if(rospy.has_param(rospy.get_name()+'/quality_criterion')):
        quality_criterion_ = rospy.get_param(
            rospy.get_name()+'/quality_criterion')
    else:
        print 'no quality_criterion supply'
        quality_criterion_ = ''
    if(rospy.has_param(rospy.get_name()+'/debug_mode')):
        debug_mode_ = rospy.get_param(rospy.get_name()+'/debug_mode')
    else:
        print 'no debug_mode supply'
        debug_mode_ = False
    intervention_times = get_bag_data(bag_dir, monitor_type)
    quality_criterion_function(quality_criterion_)
    plot(photo_path,blacklist, debug_mode_, intervention_times)

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

def get_bag_data(bag_dir, monitor_type):
    global start_time
    global value_dict
    bag = rosbag.Bag(bag_dir)
    inter_times = []

    for topic, msg, t in bag.read_messages():
        if not start_time:
            start_time = t

        if topic == '/E_VEHICLE/id0/E10_driver_intervention':
            temp = (t-start_time).to_sec()
            inter_times.append(int(temp))

        if not topic == "/monitoring":
            continue

        if monitor_type in msg.info[0].description:
            for element in msg.info[0].values:
                if  '/' in element.key:
                    revert_ = element.key[::-1]
                    num = revert_.find('/')
                    argument = element.key.replace(
                        element.key[-num:], '')
                    if  argument not in value_dict.keys():
                        value_dict[argument] = OrderedDict()
                    if argument in element.key and element.key not in value_dict[argument].keys():
                        value_dict[argument][element.key] = {'timestamp':[], 'value':[], 'unit':[], 'errorlevel':[]}
                for argument in value_dict.keys():
                    for key in value_dict[argument].keys():
                        if element.key == key:
                            temp_time = t - start_time
                            temp_time = temp_time.to_sec()
                            value_dict[argument][key]['timestamp'].append(str(temp_time))
                            value_dict[argument][key]['value'].append(str(element.value))
                            value_dict[argument][key]['unit'].append(str(element.unit))
    return inter_times

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
                ax[j - 1].text(0.9, 0.5, 'No Messages',
                               verticalalignment='top', horizontalalignment='right',
                               transform=ax[j - 1].transAxes,
                               color='blue', fontsize=10)

            if key not in tabell_errors[argument].keys():
                tabell_errors[argument][key] = value_dict[argument][key]['errorlevel']


            ax[j-1].yaxis.offsetText.set_fontsize(4)
            ax[j-1].yaxis.set_major_locator(plt.MaxNLocator(8))
            ax[j-1].xaxis.set_major_locator(plt.MaxNLocator(8))

            data_list, unit = suit_unit(value_dict[argument][key]['value'], value_dict[argument][key]['unit'][0])
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
            #if inter_times:
            #    for perc_t in inter_times:
            #        plt.axvline(x=perc_t, ymin=0.0, ymax = 0.99, linewidth=1, color='r')

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
        rospy.loginfo('%s',str(inst))

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

