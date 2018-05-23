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

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, Slot
from python_qt_binding.QtWidgets import QWidget
import rospy
import rospkg

from .timeline import Timeline

class TimelinePane(QWidget):
    """
    This class defines the pane where timeline and its related components
    are displayed.
    """

    sig_update = Signal()

    def __init__(self, parent):
        """
        Because this class is intended to be instantiated via Qt's .ui file,
        taking argument other than parent widget is not possible, which is
        ported to set_timeline_data method. That said, set_timeline_data must
        be called (soon) after an object of this is instantiated.
        """
        super(TimelinePane, self).__init__()
        self._parent = parent
        self._timeline = None

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_robot_monitor'),
                               'resource',
                               'timelinepane.ui')
        loadUi(ui_file, self)

        self._timeline_view.show()

        self.sig_update.connect(self._timeline_view.redraw)

    def set_timeline(self, timeline, name=None):
        assert(self._timeline is None)
        self._timeline = timeline

        self._timeline_view.set_timeline(timeline, name)

        # connect pause button
        self._pause_button.clicked[bool].connect(self._timeline.set_paused)
        self._timeline.pause_changed[bool].connect(
                self._pause_button.setChecked)

        # bootstrap initial state
        self._pause_button.setChecked(self._timeline.paused)
        self.sig_update.emit()

    def redraw(self):
        self.sig_update.emit()
