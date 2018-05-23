# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Isaac Saito, Ze'ev Klapow, Austin Hendrix

import os
import rospkg

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer, Signal, Qt, Slot
from python_qt_binding.QtGui import QPalette
from python_qt_binding.QtWidgets import QWidget
import rospy

from .inspector_window import InspectorWindow
from .status_item import StatusItem
from .timeline_pane import TimelinePane
from .timeline import Timeline
import rqt_robot_monitor.util_robot_monitor as util

from ros_monitoring.msg import *

class RobotMonitorWidget(QWidget):
    """
    NOTE: RobotMonitorWidget.shutdown function needs to be called
    when the instance of this class terminates.

    RobotMonitorWidget itself doesn't store previous diagnostic states.
    It instead delegates that function to Timeline class.
    """

    _TREE_ALL = 1
    _TREE_WARN = 2
    _TREE_ERR = 3

    message_updated = Signal(Gui)

    def __init__(self, context, topic=None):
        """
        :param context: plugin context hook to enable adding widgets as a
                        ROS_GUI pane, 'PluginContext'
        :param topic: Diagnostic topic to subscribe to 'str'
        """

        super(RobotMonitorWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_robot_monitor'), 'resource',
                               'robotmonitor_mainwidget.ui')
        loadUi(ui_file, self)

        obj_name = 'ROS Monitor'
        self.setObjectName(obj_name)
        self.setWindowTitle(obj_name)

        self.message_updated.connect(self.monitoring_cb)

        # if we're given a topic, create a timeline. otherwise, don't
        #  this can be used later when writing an rqt_bag plugin
        if topic:
            # create timeline data structure
            self._timeline = Timeline(topic, Gui)
            self._timeline.message_updated.connect(self.message_updated)

            # create timeline pane widget
            self.timeline_pane = TimelinePane(self)

            self.timeline_pane.set_timeline(self._timeline)

            self.vlayout_top.addWidget(self.timeline_pane)
            self.timeline_pane.show()
        else:
            self._timeline = None
            self.timeline_pane = None

        self._inspectors = {}
        # keep a copy of the current message for opening new inspectors
        self._current_msg = None

        self.tree_all_devices.itemDoubleClicked.connect(self._tree_clicked)
        self.warn_flattree.itemDoubleClicked.connect(self._tree_clicked)
        self.err_flattree.itemDoubleClicked.connect(self._tree_clicked)
        # TODO: resize on itemCollapsed and itemExpanded

        self._is_stale = False

        self._timer = QTimer()
        self._timer.timeout.connect(self._update_message_state)
        self._timer.start(1000)

        palette = self.tree_all_devices.palette()
        self._original_base_color = palette.base().color()
        self._original_alt_base_color = palette.alternateBase().color()

        self._tree = StatusItem(self.tree_all_devices.invisibleRootItem())
        self._warn_tree = StatusItem(self.warn_flattree.invisibleRootItem())
        self._err_tree = StatusItem(self.err_flattree.invisibleRootItem())

    @Slot(Gui)
    def monitoring_cb(self, msg):
        """ Gui message callback """
        self._current_msg = msg
        
        # Walk the status array and update the tree
       
        for info in msg.infos:
            
                # Compute path and walk to appropriate subtree
                path = info.name.split('/')
                if path[0] == '':
                    path = path[1:]
                tmp_tree = self._tree
                
                for p in path:
                    
                    tmp_tree = tmp_tree[p]
                
                tmp_tree.update(info, path[-1])
    
                # Check for warnings
                if int(info.errorlevel*2.9) == DiagnosticStatus.WARN: 
                    name = info.name
                    self._warn_tree[name].update(info, info.name)
    
                # Check for errors
                if int(info.errorlevel*2.9) == DiagnosticStatus.ERROR: 
                    name = info.name
                    self._err_tree[name].update(info, info.name)
            
        # For any items in the tree that were not updated, remove them
        self._tree.prune()
        self._warn_tree.prune()
        self._err_tree.prune()

        # TODO(ahendrix): implement
        # Insight: for any item that is not OK, it only provides additional
        #          information if all of it's children are OK
        #
        #          otherwise, it's just an aggregation of its children
        #          and doesn't provide any additional value when added to
        #          the warning and error flat trees

        self.tree_all_devices.resizeColumnToContents(0)
        self.warn_flattree.resizeColumnToContents(0)
        self.err_flattree.resizeColumnToContents(0)

    
    def resizeEvent(self, evt):
        """Overridden from QWidget"""
        rospy.logdebug('RobotMonitorWidget resizeEvent')
        if self.timeline_pane:
            self.timeline_pane.redraw()

    @Slot(str)
    def _inspector_closed(self, name):
        """ Called when an inspector window is closed """
        if name in self._inspectors:
            del self._inspectors[name]

    def _tree_clicked(self, item, column):
        """
        Slot to QTreeWidget.itemDoubleClicked

        :type item: QTreeWidgetItem
        :type column: int
        """
        rospy.logdebug('RobotMonitorWidget _tree_clicked col=%d', column)
        if item.name in self._inspectors:
            self._inspectors[item.name].activateWindow()
        else:
            self._inspectors[item.name] = InspectorWindow(self, item.name,
                    self._current_msg, self._timeline)
            self._inspectors[item.name].closed.connect(self._inspector_closed)
            self.message_updated.connect(self._inspectors[item.name].message_updated)

    def _update_message_state(self):
        """ Update the display if it's stale """
        if self._timeline is not None:
            if self._timeline.has_messages:
                previous_stale_state = self._is_stale
                self._is_stale = self._timeline.is_stale

                time_diff = int(self._timeline.data_age())

                msg_template = "Last message received %s %s ago"
                if time_diff == 1:
                    msg = msg_template % (time_diff, "second")
                else:
                    msg = msg_template % (time_diff, "seconds")
                self.timeline_pane._msg_label.setText(msg)
                if previous_stale_state != self._is_stale:
                    self._update_background_color()
            else:
                # no messages received yet
                self.timeline_pane._msg_label.setText("No messages received")

    def _update_background_color(self):
        """ Update the background color based on staleness """
        p = self.tree_all_devices.palette()
        if self._is_stale:
            p.setColor(QPalette.Base, Qt.darkGray)
            p.setColor(QPalette.AlternateBase, Qt.lightGray)
        else:
            p.setColor(QPalette.Base, self._original_base_color)
            p.setColor(QPalette.AlternateBase, self._original_alt_base_color)
        self.tree_all_devices.setPalette(p)
        self.warn_flattree.setPalette(p)
        self.err_flattree.setPalette(p)

    def shutdown(self):
        """
        This needs to be called whenever this class terminates.
        This closes all the instances on all trees.
        Also unregisters ROS' subscriber, stops timer.
        """
        rospy.logdebug('RobotMonitorWidget in shutdown')

        names = self._inspectors.keys()
        for name in names:
            self._inspectors[name].close()

        if self._timeline:
            self._timeline.shutdown()

        self._timer.stop()
        del self._timer

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('splitter', self.splitter.saveState())
        # TODO(ahendrix): persist the device paths, positions and sizes of any
        #                 inspector windows

    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.contains('splitter'):
            self.splitter.restoreState(instance_settings.value('splitter'))
        else:
            self.splitter.setSizes([100, 100, 200])
        # TODO(ahendrix): restore inspector windows
