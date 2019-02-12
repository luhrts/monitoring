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
from monitoring_msgs.msg import *
from monitoring_core.monitor import Monitor
from monitoring_core.monitor import AggregationStrategies
from rosgraph_msgs.msg import TopicStatistics

from subprocess import check_output
from rospy.msg import AnyMsg

ID = "TOPICSTATISTICS"
MONITOR_ = Monitor("statistics_monitor")
monitor_mode = 1
BIG_VALUE = 10000000.0


def init():
    """
    Init rospy node
    check for frequency parameter (default = 1Hz)
    check for filter_type (0 = default (list all), 1 = whitelist, 2 = black list)
    Return: frequency and filter_type, list_of_topic_names
    """
    topics = []
    if rospy.has_param('statistics_monitor/monitor_mode'):
        monitor_mode = rospy.get_param('statistics_monitor/monitor_mode')
    else:
        monitor_mode = 1
    if rospy.has_param('statistics_monitor/frequency'):
        frequency = rospy.get_param('statistics_monitor/frequency')
    else:
        frequency = 1
    if rospy.has_param('statistics_monitor/filter_type'):
        filter_type = rospy.get_param('statistics_monitor/filter_type')
        if filter_type == "WHITELIST":
            filter_type = Filter_type.WHITELIST
        elif filter_type == "BLACKLIST":
            filter_type = Filter_type.BLACKLIST
        else:
            filter_type = Filter_type.DEFAULT
    else:
        filter_type = Filter_type.DEFAULT
    if filter_type != Filter_type.DEFAULT:
        try:
            topics = rospy.get_param('statistics_monitor/topics')
        except Exception as e:
            print e
    return frequency, filter_type, topics

def load_params(topic_names):
    configs = []
    for topic in topic_names:
        if topic != 'DEFAULT':
            c = Topic_config(topic)
            if rospy.has_param(topic+'/destination'):
                c.destinations = rospy.get_param(topic+'/destination')
            else:
                c.destinations = -1
            if rospy.has_param(topic+'/source'):
                c.sources = rospy.get_param(topic+'/source')
            else:
                c.sources = -1
            if rospy.has_param(topic+'/frequency'):
                c.frequency = rospy.get_param(topic+'/frequency')
            else:
                c.frequency = -1
            if rospy.has_param(topic+'/dfrequency'):
                c.dfrequency = rospy.get_param(topic+'/dfrequency')
            else:
                c.dfrequency = -1
            if rospy.has_param(topic+'/size'):
                c.size = rospy.get_param(topic+'/size')
            else:
                c.size = -1
            if rospy.has_param(topic+'/dsize'):
                c.dsize = rospy.get_param(topic+'/dsize')
            else:
                c.dsize = -1
            if rospy.has_param(topic+'/stampmaxage'):
                c.stampmaxage = rospy.get_param(topic+'/stampmaxage')
            else:
                c.stampmaxage = -1
            if rospy.has_param(topic+'/errorlevel'):
                c.errorlevel = rospy.get_param(topic+'/errorlevel')
            else:
                c.errorlevel = -1
            configs.append(c)
        else:
            if rospy.has_param(topic+'/destination'):
                Config_default.DESTINATIONS = rospy.get_param(topic+'/destination')
            if rospy.has_param(topic+'/source'):
                Config_default.SOURCES = rospy.get_param(topic+'/source')
            if rospy.has_param(topic+'/frequency'):
                Config_default.FREQUENZY = rospy.get_param(topic+'/frequency')
            if rospy.has_param(topic+'/dfrequency'):
                Config_default.DFREQUENZY = rospy.get_param(topic+'/dfrequency')
            if rospy.has_param(topic+'/size'):
                Config_default.SIZE = rospy.get_param(topic+'/size')
            if rospy.has_param(topic+'/dsize'):
                Config_default.DSIZE = rospy.get_param(topic+'/dsize')
            if rospy.has_param(topic+'/stampmaxage'):
                Config_default.STAMPMAXAGE = rospy.get_param(topic+'/stampmaxage')
            if rospy.has_param(topic+'/errorlevel'):
                Config_default.ERRORLEVEL = rospy.get_param(topic+'/errorlevel')
    return configs




class Filter_type(object):
    DEFAULT = 0
    WHITELIST = 1
    BLACKLIST = 2

class Config_default():
    SOURCES = 1
    DESTINATIONS = -1
    FREQUENZY = -1
    DFREQUENZY = -1
    SIZE = -1
    DSIZE = -1
    STAMPMAXAGE = 0.1
    ERRORLEVEL = 0.2

class Topic_config():
    def __init__(self, name):
        self.name = name
        self.sources =  []
        self.destinations = []
        self.frequency = None
        self.dfrequenzy = None
        self.size = None
        self.dsize = None
        self.stampmaxage = None
        self.errorlevel = None

class TopicsStatisticsAnalyzer:
    def __init__(self, Filter_type, monitor, config):
        if rospy.has_param('statistics_monitor/ignore_default'):
            self.ignore_default = rospy.get_param('statistics_monitor/ignore_default')
        else:
            self.ignore_default = 0
        self.config = config
        self.filter_type = Filter_type
        self.data = {}
        self.time = rospy.Time.now()
        self.monitor = monitor
        self.sub = rospy.Subscriber("/statistics",
                                    TopicStatistics, self.statistics_cb)

        #self.check_unsubscriberd_topics_timer = rospy.Timer(rospy.Duration(4.0), self.timer_cb)
        self.active_topics = []
        self.inactive_topics_new = []
        self.inactive_topics_old =  []
        self.starttime = rospy.Time.now()
        self.subscribed_topics = []

    def timer_cb(self, timer):
        '''
        This Function subscribes to a topic if there are no information 
        about it in topic statistics. This should produce at least some infos
        about the topic.
        After Start it will wait 20 secondes.
        Usefull if enable_statistics was not set before the ros system started
        '''
        # TODO Test me properly
        time = rospy.Time.now()
        if (time - self.starttime).to_sec() < 10.0:
            return

        topics = check_output('rostopic list', shell=True).split('\n')

        for t in topics:
            if t in self.active_topics:
                # already statistics msgs recieved
                continue
            elif t in self.inactive_topics_old:
                # already checked, dont need to subscribe
                continue
            elif t in self.subscribed_topics:
                # already subscribed
                continue
            if not t:
                continue
            sub = rospy.Subscriber(t, AnyMsg, self.any_cb)
            self.subscribed_topics.append(t)
            self.inactive_topics_new.append((t, sub, time))

        for top, subs, ti in self.inactive_topics_new:
            if (time - ti).to_sec() < 19.9:
                continue
            # unsubscribe after a period of time
            subs.unregister()
            self.inactive_topics_old.append(top)


    def any_cb(self, msg):
        return

    def inconfig(self, name):
        for tp in self.config:
            n = tp.name
            #rospy.loginfo(n)
            if tp.name == name:
                return True
        return False

    def statistics_cb(self, msg):
        name = msg.topic
        if self.filter_type == Filter_type.WHITELIST:
            #rospy.loginfo(name)
            if not self.inconfig(name):
                return

        if self.filter_type == Filter_type.BLACKLIST:
            if self.inconfig(name):
                return
        if name not in self.active_topics:
            self.active_topics.append(name)
        name = name.replace("/","|")
        key_pub = msg.node_pub.replace("/","|")
        key_sub = msg.node_sub.replace("/","|")
        top_sub = key_pub+"--->"+key_sub
        start = msg.window_start
        stop = msg.window_stop
        if name not in self.data:
            self.data[name] = {}
            self.data[name]["publisher"] = []
            self.data[name]["subscriber"] = []
        if not key_pub in self.data[name]["publisher"]:
            self.data[name]["publisher"].append(key_pub)
        if not key_sub in self.data[name]["subscriber"]:
            self.data[name]["subscriber"].append(key_sub)
        if top_sub not in self.data[name]:
            # start, byte/msg, stop, max_bw, min_bw, total_time, total_traffic, avg_bw, delivered, dropped, cur_delivered, cur_dropped, frequence
            self.data[name][top_sub] = [0,0,0,0,BIG_VALUE,0.0,0.0,0.0,0,0,0,0,0.0,0,0.0]
            self.data[name][top_sub][0] = start.to_sec()
        self.data[name][top_sub][1] = float(msg.traffic) / (msg.dropped_msgs + msg.delivered_msgs)
        self.data[name][top_sub][2] = stop.to_sec()
        bw = msg.traffic / (stop - start).to_sec()
        if bw > self.data[name][top_sub][3]:
            self.data[name][top_sub][3] = bw
        if bw < self.data[name][top_sub][4]:
            self.data[name][top_sub][4] = bw
        self.data[name][top_sub][5] += (stop - start).to_sec()
        self.data[name][top_sub][6] += msg.traffic
        self.data[name][top_sub][7] = self.data[name][top_sub][6] / self.data[name][top_sub][5]
        self.data[name][top_sub][8] += msg.delivered_msgs
        self.data[name][top_sub][9] += msg.dropped_msgs
        self.data[name][top_sub][1] = float(self.data[name][top_sub][6]) / (self.data[name][top_sub][8] + self.data[name][top_sub][9])
        self.data[name][top_sub][10] = msg.delivered_msgs
        self.data[name][top_sub][11] = msg.dropped_msgs
        self.data[name][top_sub][12] = (self.data[name][top_sub][8] + self.data[name][top_sub][9]) / (stop - start).to_sec()
        self.data[name][top_sub][13] = msg.stamp_age_mean.to_sec()
        if msg.period_mean.to_sec() > 0.0:
            self.data[name][top_sub][14] = 1 / msg.period_mean.to_sec()

    def get_infos_and_publish(self):
        # key1 e.g /B_CONTROLLER/id0/G00_data_analysis_vector
        # key2 e.g /VehicleController_id0--->/B01_Type01 
        for key1 in self.data.keys():
            for key2 in self.data[key1].keys():
                err = 0.0
                if key2 == 'subscriber':
                    err = self.calcerror("subscribers", [key1], self.data[key1]['subscriber'])
                    VALUE_DICT['subscribers'](key1, self.data[key1]['subscriber'], err)
                    VALUE_DICT['num_subscribers'](key1, len(self.data[key1]['subscriber']),err)
                    continue
                if key2 == 'publisher':
                    err = self.calcerror("publishers", [key1], self.data[key1]['publisher'])
                    VALUE_DICT['publishers'](key1, self.data[key1]['publisher'],err)
                    VALUE_DICT['num_publishers'](key1, len(self.data[key1]['publisher']),err)
                    continue
                err = 0.0
                VALUE_DICT['byte_per_msg'](key1 +"/"+ key2, self.data[key1][key2][1], 0)
                VALUE_DICT['avg_bandwidth'](key1 +"/"+ key2, self.data[key1][key2][7], 0)
                VALUE_DICT['count_msg_delivered'](key1 +"/"+ key2, self.data[key1][key2][8], 0)
                VALUE_DICT['count_msg_dropped'](key1 +"/"+ key2, self.data[key1][key2][9], 0)
                VALUE_DICT['cur_delivered'](key1 +"/"+ key2, self.data[key1][key2][10], 0)
                err = self.calcerror("dropped", [key1,key2], self.data[key1][key2][11])
                VALUE_DICT['cur_dropped'](key1 +"/"+ key2, self.data[key1][key2][11], err)
                VALUE_DICT['bw_max'](key1 +"/"+ key2, self.data[key1][key2][3], 0)
                VALUE_DICT['bw_min'](key1 +"/"+ key2, self.data[key1][key2][4], 0)
                VALUE_DICT['total_traffic'](key1 +"/"+ key2, self.data[key1][key2][6], 0)
                if self.data[key1][key2][14] > 0:
                    err = self.calcerror("frequency", [key1, key2], self.data[key1][key2][14])
                    VALUE_DICT['frequency'](key1 +"/"+ key2, self.data[key1][key2][14], err)
                err = self.calcerror("stampmaxage", [key1, key2],self.data[key1][key2][13])
                VALUE_DICT['stamp_age'](key1 +"/"+ key2, self.data[key1][key2][13], err)


    def calcerror(self, key, topics, value):
        #rospy.loginfo(topic)
	return 0.0
        topic = topics[0].replace("|","/")
        cval, baseerr = 0.0, 0.0
        if self.inconfig(topic):
            cval, baseerr = self.getconfigval(topic, key)
        elif not self.inconfig(topic) and self.ignore_default == 0:
            cval, baseerr = self.getdefaultval(key)
        else:
            return 0.0

        if key == "publishers":
            return self.subpuberr(topic, value, cval, baseerr, key)
        if key == "subscribers":
            return self.subpuberr(topic, value, cval, baseerr, key)
        if key == "frequency":
            return self.frequencyerr(value, cval, baseerr)
        if key == "size":
            pass
        if key == 'dropped':
            return self.droppederr(value, cval, baseerr)
        if key == "stampmaxage":
            return self.stampageerr(value, cval, baseerr)
        return 0.0

    def droppederr(self, value, cval, baseerr):
        return baseerr

    def stampageerr(self, value, cval, baseerr):
        if cval == -1:
            return 0.0
        if value <= cval:
            return 0.0
        fac = value / cval
        if fac*baseerr >= 1.0:
            return 1.0
        return fac*baseerr

    def frequencyerr(self, value, cval, baseerr):
        f, df = cval[0], cval[1]

        if f == -1:
            return 0.0
        delta = abs(value - f)
        if delta <= df:
            return 0.0
        else:
            fac = delta / df
            if fac*baseerr >= 1.0:
                return 1.0
            return fac*baseerr

    def subpuberr(self, topic, value, cval, baseerr, key):
        if type(cval) is int:
            if cval < 0:
                return 0.0
            elif cval == 0:
                rospy.logwarn("0 %s for topic: %s, this might be wrong!", key, topic)
                return 1.0
            else:
                fac = float(len(value) / cval)
                if fac < 1.0:
                    return 0.0
                else:
                    return fac * baseerr
        if type(cval) is list:
            wrong = 0
            right = 0
            for t in value:
                if t in cval:
                    right +=1
                else:
                    wrong +=1
            fac = wrong * baseerr
            if fac > 1.0:
                return 1.0
            else:
                return fac

    def getconfigval(self, topic, key):
        for tp in self.config:
            if tp.name == topic:
                #rospy.loginfo(topic)
                #rospy.loginfo(tp.name)
                if key == "publishers":
                    return tp.sources, tp.errorlevel
                if key == "subscribers":
                    return tp.destinations, tp.errorlevel
                if key == "frequency":
                    return [tp.frequency, tp.dfrequency], tp.errorlevel
                if key == "size":
                    return [tp.size, tp.dsize], tp.errorlevel
                if key == "stampmaxage":
                    return tp.stampmaxage, tp.errorlevel
                if key == "dropped":
                    return 0.0, tp.errorlevel

    def getdefaultval(self, key):
        if key == "publishers":
            return Config_default.SOURCES, Config_default.ERRORLEVEL
        if key == "subscribers":
            return Config_default.DESTINATIONS, Config_default.ERRORLEVEL
        if key == "frequency":
            return [Config_default.FREQUENZY, Config_default.DFREQUENZY], Config_default.ERRORLEVEL
        if key == "size":
            return [Config_default.SIZE, Config_default.DSIZE], Config_default.ERRORLEVEL
        if key == "stampmaxage":
            return Config_default.STAMPMAXAGE, Config_default.ERRORLEVEL
        if key == "dropped":
            return 0.0, Config_default.ERRORLEVEL

def byte_per_msg_to_monitor(name, value, err):
    monitor_string = name + "/byte_per_msg"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0
    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def avg_bandwidth_to_monitor(name, value, err):
    monitor_string = name + "/avg_bandwidth"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0
    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def count_delivered_to_monitor(name, value, err):
    monitor_string = name + "/total_msg_delivered"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0
    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def count_dropped_to_monitor(name, value, err):
    monitor_string = name + "/total_msg_dropped"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0
    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def current_delivered_to_monitor(name, value, err):
    monitor_string = name + "/current_msg_delivered"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0
    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def current_dropped_to_monitor(name, value, err):
    monitor_string = name + "/current_msg_dropped"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0
    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def max_bandwidth_to_monitor(name, value, err):
    monitor_string = name + "/max_bandwidth"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0
    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def min_bandwidth_to_monitor(name, value, err):
    monitor_string = name + "/min_bandwidth"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = 0
    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def total_traffic_to_monitor(name, value, err):
    monitor_string = name + "/total_bytes_traffic"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = err
    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def subribers_to_monitor(name, value, err):
    monitor_string = name + "/subscriber"
    monitor_value = ''
    for v in value:
        monitor_value = monitor_value + v + ", "
    monitor_unit = " "
    monitor_errorlvl = err
    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def num_subscribers_to_monitor(name, value, err):
    monitor_string = name + "/num_subscriber"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = err
    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def num_publishers_to_monitor(name, value, err):
    monitor_string = name + "/num_publisher"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = err
    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def publishers_to_monitor(name, value, err):
    monitor_string = name + "/publisher"
    monitor_value = ''
    for v in value:
        monitor_value = monitor_value + v + ", "
    monitor_unit = " "
    monitor_errorlvl = err
    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def stamp_age_to_monitor(name, value, err):
    monitor_string = name + "/stamp_age"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = err
    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

def frequency_to_monitor(name, value, err):
    monitor_string = name + "/frequency"
    monitor_value = str(value)
    monitor_unit = " "
    monitor_errorlvl = err
    MONITOR_.addValue(monitor_string, monitor_value, monitor_unit, monitor_errorlvl, monitor_mode)

VALUE_DICT={
    'byte_per_msg' : byte_per_msg_to_monitor,
    'avg_bandwidth' : avg_bandwidth_to_monitor,
    'count_msg_delivered' : count_delivered_to_monitor,
    'count_msg_dropped' : count_dropped_to_monitor,
    'cur_delivered' : current_delivered_to_monitor,
    'cur_dropped' : current_dropped_to_monitor,
    'bw_max' : max_bandwidth_to_monitor,
    'bw_min' : min_bandwidth_to_monitor,
    'total_traffic' : total_traffic_to_monitor,
    'subscribers': subribers_to_monitor,
    'num_subscribers': num_subscribers_to_monitor,
    'publishers': publishers_to_monitor,
    'num_publishers': num_publishers_to_monitor,
    'stamp_age': stamp_age_to_monitor,
    'frequency': frequency_to_monitor
}


if __name__ == '__main__':
    rospy.init_node("statistics_monitor")
    FREQUENCY, FILTER_TYPE_, TOPIC_NAMES_ = init()

    CONFIG_ = load_params(TOPIC_NAMES_)

    rospy.loginfo(FREQUENCY)
    rospy.loginfo(FILTER_TYPE_)
    RATE = rospy.Rate(FREQUENCY)
    MONITOR_ = Monitor("statistics_monitor")

    tsa_object = TopicsStatisticsAnalyzer(FILTER_TYPE_, MONITOR_, CONFIG_)

    while not rospy.is_shutdown():
        try:
            tsa_object.get_infos_and_publish()
        except rospy.ROSInterruptException:
            rospy.loginfo("ERROR")
        except Exception as e:
            print e
        finally:
            RATE.sleep()

