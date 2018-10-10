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

from monitoring_msgs.msg import *

class PlotWidget(QWidget):
    _redraw_interval = 40
    list_of_elements_in_plot = []
    monitoring_topic_overview_list = []

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

        rospy.Subscriber("/monitoring/gui", Gui, self.monitoring_topic_overview_list_manager)
        
    def monitoring_topic_overview_list_manager(self, msg):
        
        for element in msg.infos:
            if not element.name in self.monitoring_topic_overview_list:
                self.monitoring_topic_overview_list.append(element.name)

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
        for element in self.monitoring_topic_overview_list:
            self.listWidget.addItem(element)

    @Slot()
    def on_subscribe_topic_button_clicked(self):
        node_value_name = self.listWidget.currentItem().text()
        if node_value_name not in self.list_of_elements_in_plot:
            self.add_topic(node_value_name)

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
            for topic_name, rosdata in self._rosdata.items():
                try:
                    data_x, data_y = rosdata.next()
                    if data_x or data_y:
                        rospy.logout(data_x)
                        rospy.logout(data_y)
                        self.data_plot.update_values(topic_name, data_x, data_y)
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

    def add_topic(self, node_value_name):
        topics_changed = False
        rospy.logwarn("new value with name: %s", node_value_name)
        self.list_of_elements_in_plot.append(node_value_name)
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
        if topic_name in self.list_of_elements_in_plot:
                self.list_of_elements_in_plot.remove(topic_name)
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
            if topic_name in self.list_of_elements_in_plot:
                print topic_name
                self.list_of_elements_in_plot.remove(topic_name)
        self._rosdata = {}

        self._subscribed_topics_changed()

    def enable_timer(self, enabled=True):
        if enabled:
            rospy.logout("Timer enabled")
            self._update_plot_timer.start(self._redraw_interval)
        else:
            rospy.logout("Timer stopped")
            self._update_plot_timer.stop()
