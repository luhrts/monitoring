#!/usr/bin/env python

"""
Original Copyright notice: see below
Modified by Michael Lange, Leibniz Universitaet Hannover, 2018
This program is a modified version of "rqt_plot" by Dorian Scholz.
It will now display a list of all values published under /monitoring topic.
The user may select items from the list and add them to the plot.
Errors are thrown if a value is not convertable to float.
"""

# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import rospkg
import roslib

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QAction, QMenu, QWidget

import rospy

from . rosplot import ROSData, RosPlotException

from std_msgs.msg import String
from monitoring_msgs.msg import *

monitoring_topic_overview_list = []
testlist = []

def monitoring_listener():
    """
    retrieve just one message from the monitoring topic,
    containing the MonitoringArray
    """
    msg = rospy.wait_for_message("monitoring", MonitoringArray)
    return msg

def monitoring_topic_overview_list_manager(msg):
    global monitoring_topic_msg
    monitoring_topic_msg = msg
    for element in msg.infos:
        if not element.name in monitoring_topic_overview_list:
            monitoring_topic_overview_list.append(element.name)
            rospy.loginfo(element.name)
            #testp, testn = get_topic_name(element.name)
            #rospy.loginfo("testp: %s", testp)
            testlist.append({'name':element.name,'path': element.name})

def get_topic_name(node_value_name):
    """
    retrieve the absolut path to a topic within the MonitoringArray,
    by searching for the node_value_name and retrieving its position within the Array
    """
    msg = monitoring_topic_msg
    i = 0
    for element in msg.infos:
        if node_value_name == str(element.name):
            topic_absolute_path = "/monitoring/gui/infos[%d]/value" % i
            return topic_absolute_path, node_value_name
        i = i + 1

def get_node_value_name(topic_name):
    """
    Oposing function to get_topic_name
    get absolute path in MonitoringArray to topic,
    convert the string to access the msg element at the correct spot
    and retrieve the node_value_name
    """
    msg = monitoring_listener()
    temp = str(topic_name)
    temp = temp.replace("value","key")
    temp = temp.replace("keys", "values")
    temp = temp.replace("/monitoring/","msg.")
    temp = temp.replace("/",".")
    temp_eval = eval(temp)
    node_value_name = temp_eval
    return(node_value_name)

def check_value_existence(topic_absolute_path, node_value_name):
 #   rospy.loginfo("check existence - topic: %s node: %s", topic_absolute_path, node_value_name)
#    temp_str = topic_absolute_path
#    temp_str = temp_str.replace("value","name")
#    temp_str = temp_str.replace("/monitoring/gui/","monitoring_topic_msg.")
#    temp_str = temp_str.replace("/",".")
#    temp_name = eval(temp_str)
    if topic_absolute_path == node_value_name:
        return True
    else:
        return False

class PlotWidget(QWidget):
    _redraw_interval = 40
    list_of_elements_in_plot = []

    def __init__(self, initial_topics=None, start_paused=False):
        super(PlotWidget, self).__init__()
        self.setObjectName('PlotWidget')
        self._initial_topics = initial_topics

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('monitoring_rqt_plot'), 'resource', 'plot.ui')
        loadUi(ui_file, self)
        self.subscribe_topic_button.setIcon(QIcon.fromTheme('list-add'))
        self.remove_topic_button.setIcon(QIcon.fromTheme('list-remove'))
        self.pause_button.setIcon(QIcon.fromTheme('media-playback-pause'))
        self.clear_button.setIcon(QIcon.fromTheme('edit-clear'))
        self.data_plot = None
        self.subscribe_topic_button.setEnabled(True)
        if start_paused:
            self.pause_button.setChecked(True)

        self._start_time = rospy.get_time()
        self._rosdata = {}
        self._remove_topic_menu = QMenu()

        # init and start update timer for plot
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)

        rospy.Subscriber("/monitoring/gui", Gui, monitoring_topic_overview_list_manager)

    def switch_data_plot_widget(self, data_plot):

        self.enable_timer(enabled=False)
        self.data_plot_layout.removeWidget(self.data_plot)
        if self.data_plot is not None:
            self.data_plot.close()
        self.data_plot = data_plot
        self.data_plot_layout.addWidget(self.data_plot)
        self.data_plot.autoscroll(self.autoscroll_checkbox.isChecked())

        self._subscribed_topics_changed()

    @Slot()
    def on_refresh_list_button_clicked(self):
        self.listWidget.clear()
        """
        msg = monitoring_listener()
        for element in msg.info[0].values:
            self.listWidget.addItem(element.key)
        """
        for element in monitoring_topic_overview_list:
            self.listWidget.addItem(element)

    @Slot()
    def on_subscribe_topic_button_clicked(self):
        topic_absolute_path, node_value_name = get_topic_name(self.listWidget.currentItem().text())
        #topic_name = self.listWidget.currentItem().text()
        if topic_absolute_path not in self.list_of_elements_in_plot:
            self.list_of_elements_in_plot.append(topic_absolute_path)
            self.add_topic(topic_absolute_path, node_value_name)

    @Slot(bool)
    def on_pause_button_clicked(self, checked):
        self.enable_timer(not checked)

    @Slot(bool)
    def on_autoscroll_checkbox_clicked(self, checked):
        self.data_plot.autoscroll(checked)
        if checked:
            self.data_plot.redraw()

    @Slot()
    def on_clear_button_clicked(self):
        self.clear_plot()

    def update_plot(self):

        if self.data_plot is not None:
            needs_redraw = False
            for topic_absolute_path, rosdata in self._rosdata.items():
                for element in testlist:
                    if element['path'] == topic_absolute_path:
                        if check_value_existence(topic_absolute_path, element['name']):
                            try:
                                data_x, data_y = rosdata.next()
                                if data_x or data_y:
                                    rospy.logout(data_x)
                                    rospy.logout(data_y)
                                    self.data_plot.update_values(topic_absolute_path, data_x, data_y)
                                    needs_redraw = True
                            except RosPlotException as e:
                                qWarning('PlotWidget.update_plot(): error in rosplot: %s' % e)
            if needs_redraw:
                self.data_plot.redraw()

    def _subscribed_topics_changed(self):
        self._update_remove_topic_menu()
        rospy.logout("_subscribed_topics_changed")
        if not self.pause_button.isChecked():
            rospy.logout("enable timer")
            # if pause button is not pressed, enable timer based on subscribed topics
            self.enable_timer(self._rosdata)
        self.data_plot.redraw()

    def _update_remove_topic_menu(self):
        def make_remove_topic_function(x):
            return lambda: self.remove_topic(x)

        self._remove_topic_menu.clear()
        for topic_name in sorted(self._rosdata.keys()):
            node_value_name = topic_name
            action = QAction(node_value_name, self._remove_topic_menu)
            action.triggered.connect(make_remove_topic_function(topic_name))
            self._remove_topic_menu.addAction(action)

        if len(self._rosdata) > 1:
            all_action = QAction('All', self._remove_topic_menu)
            all_action.triggered.connect(self.clean_up_subscribers)
            self._remove_topic_menu.addAction(all_action)

        self.remove_topic_button.setMenu(self._remove_topic_menu)

    def add_topic(self, topic_absolute_path, node_value_name):
        topics_changed = False
        rospy.logwarn("new value with path: %s and name: %s", topic_absolute_path, node_value_name)
        self._rosdata[node_value_name] = ROSData(node_value_name, self._start_time)
        data_x, data_y = self._rosdata[node_value_name].next()
        self.data_plot.add_curve(node_value_name, node_value_name, data_x, data_y)
        topics_changed = True
        if topics_changed:
            self._subscribed_topics_changed()

    def remove_topic(self, topic_name):
        rospy.logout("remove topic")
        self._rosdata[topic_name].close()
        del self._rosdata[topic_name]
        self.data_plot.remove_curve(topic_name)
        rospy.logout("remove " + str(topic_name))
        #rospy.logout(list_of_elements_in_plot)
        #self.list_of_elements_in_plot.remove(topic_name)
        self._subscribed_topics_changed()

    def clear_plot(self):
        rospy.logout("clear plot")
        for topic_name, _ in self._rosdata.items():
            self.data_plot.clear_values(topic_name)
        self.data_plot.redraw()

    def clean_up_subscribers(self):
        rospy.logout("clean up subscribers")
        for topic_name, rosdata in self._rosdata.items():
            rosdata.close()
            self.data_plot.remove_curve(topic_name)
        self._rosdata = {}

        self._subscribed_topics_changed()

    def enable_timer(self, enabled=True):
        if enabled:
            rospy.logout("Timer enabled")
            self._update_plot_timer.start(self._redraw_interval)
        else:
            rospy.logout("Timer stopped")
            self._update_plot_timer.stop()
