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

from python_qt_binding.QtCore import Signal, Slot
from python_qt_binding.QtWidgets import QPushButton, QTextEdit, QVBoxLayout, QWidget
import rospy

from .status_snapshot import StatusSnapshot, level_to_text
from .timeline_pane import TimelinePane
import rqt_robot_monitor.util_robot_monitor as util


from monitoring_msgs.msg import *


class InspectorWindow(QWidget):
    closed = Signal(str)

    def __init__(self, parent, name, info, timeline):
        """
        :type status: DiagnosticStatus
        :param close_callback: When the instance of this class
                               (InspectorWindow) terminates, this callback gets
                               called.
        """
        #TODO(Isaac) UI construction that currently is done in this method,
        #            needs to be done in .ui file.

        super(InspectorWindow, self).__init__()
        self.setWindowTitle(name)
        self._name = name

        self.layout_vertical = QVBoxLayout(self)

        self.disp = StatusSnapshot(parent=self)

        self.layout_vertical.addWidget(self.disp, 1)

        if timeline is not None:
            self.timeline_pane = TimelinePane(self)
            self.timeline_pane.set_timeline(timeline, name)
            self.layout_vertical.addWidget(self.timeline_pane, 0)

            self.snapshot = QPushButton("Snapshot")
            self.snapshot.clicked.connect(self._take_snapshot)
            self.layout_vertical.addWidget(self.snapshot)

        self.snaps = []

        self.setLayout(self.layout_vertical)
        # TODO better to be configurable where to appear.
        self.resize(400, 600)
        self.show()
        self.message_updated(info)

    def closeEvent(self, event):
        """ called when this window is closed

        Calls close on all snapshots, and emits the closed signal
        """
        # TODO: are snapshots kept around even after they're closed?
        #       this appears to work even if the user closes some snapshots,
        #       and they're still left in the internal array
        for snap in self.snaps:
            snap.close()
        self.closed.emit(self._name)

    @Slot(Gui)
    def message_updated(self, msg):
        info = util.get_info_by_name(msg, self._name)
        if info != None:
            scroll_value = self.disp.verticalScrollBar().value()

            rospy.logdebug('InspectorWin message_updated')

            self.info = info
            self.disp.write_status.emit(info) #evtl info

            if self.disp.verticalScrollBar().maximum() < scroll_value:
                scroll_value = self.disp.verticalScrollBar().maximum()
            self.disp.verticalScrollBar().setValue(scroll_value)

    def _take_snapshot(self):
        snap = StatusSnapshot(status=self.info)
        self.snaps.append(snap)
