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

from math import floor
from collections import deque
import rospy

from python_qt_binding.QtCore import QPointF, Signal, Slot
from python_qt_binding.QtGui import QColor, QIcon
from python_qt_binding.QtWidgets import QGraphicsPixmapItem, QGraphicsView, \
    QGraphicsScene

import rqt_robot_monitor.util_robot_monitor as util
from ros_monitoring.msg import *


class TimelineView(QGraphicsView):
    """
    This class draws a graphical representation of a timeline.

    This is ONLY the bar and colored boxes.

    When you instantiate this class, do NOT forget to call set_init_data to
    set necessary data.
    """

    redraw = Signal()

    def __init__(self, parent):
        """Cannot take args other than parent due to loadUi limitation."""

        super(TimelineView, self).__init__()
        self._parent = parent
        self._timeline_marker = QIcon.fromTheme('system-search')

        self._min = 0
        self._max = 0
        self._xpos_marker = 5

        self._timeline_marker_width = 15
        self._timeline_marker_height = 15
        self._last_marker_at = 2

        self.redraw.connect(self._slot_redraw)

        self._timeline = None
        self._queue = deque(maxlen=30)

        self.setUpdatesEnabled(True)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)

    def set_timeline(self, timeline, name=None):
        assert(self._timeline is None)
        self._name = name
        self._timeline = timeline
        self._queue = self._timeline.queue
        self._timeline.queue_updated.connect(self._updated)

    @Slot(deque)
    def _updated(self, queue):
        """
        Update the widget whenever we receive a new message
        """

        # update timeline queue
        self._queue = queue

        # redraw
        self.redraw.emit()

    def mouseReleaseEvent(self, event):
        """
        :type event: QMouseEvent
        """
        xpos = self.pos_from_x(event.x())
        self.set_marker_pos(xpos)

    def mousePressEvent(self, event):
        """
        :type event: QMouseEvent
        """
        assert(self._timeline is not None)
        # Pause the timeline
        self._timeline.set_paused(True)

        xpos = self.pos_from_x(event.x())
        self.set_marker_pos(xpos)

    def mouseMoveEvent(self, event):
        """
        :type event: QMouseEvent
        """
        xpos = self.pos_from_x(event.x())
        self.set_marker_pos(xpos)

    def pos_from_x(self, x):
        """
        Get the index in the timeline from the mouse click position

        :param x: Position relative to self widget.
        :return: Index
        """
        width = self.size().width()
        # determine value from mouse click
        width_cell = width / float(max(len(self._timeline), 1))
        return int(floor(x / width_cell))

    def set_marker_pos(self, xpos):
        """
        Set marker position from index

        :param xpos: Marker index
        """
        assert(self._timeline is not None)
        self._xpos_marker = self._clamp(xpos, self._min, self._max)

        if self._xpos_marker == self._last_marker_at:
            # Clicked the same pos as last time.
            return
        elif self._xpos_marker >= len(self._timeline):
            # When clicked out-of-region
            return

        self._last_marker_at = self._xpos_marker

        # Set timeline position. This broadcasts the message at that position
        # to all of the other viewers
        self._timeline.set_position(self._xpos_marker)

        # redraw
        self.redraw.emit()

    def _clamp(self, val, min, max):
        """
        Judge if val is within the range given by min & max.
        If not, return either min or max.

        :type val: any number format
        :type min: any number format
        :type max: any number format
        :rtype: int
        """
        if (val < min):
            return min
        if (val > max):
            return max
        return val

    @Slot()
    def _slot_redraw(self):
        """
        Gets called either when new msg comes in or when marker is moved by
        user.
        """
        # update the limits
        self._min = 0
        self._max = len(self._timeline)-1

        # update the marker position
        self._xpos_marker = self._timeline.get_position()

        self._scene.clear()

        qsize = self.size()
        width_tl = qsize.width()

        w = width_tl / float(max(len(self._timeline), 1))
        is_enabled = self.isEnabled()

        if self._timeline is not None:
            for i, m in enumerate(self._queue):
                h = self.viewport().height()

                # Figure out each cell's color.
                qcolor = QColor('grey')
                if is_enabled:
                    qcolor = self._get_color_for_value(m)

#  TODO Use this code for adding gradation to the cell color.
#                end_color = QColor(0.5 * QColor('red').value(),
#                                   0.5 * QColor('green').value(),
#                                   0.5 * QColor('blue').value())

                self._scene.addRect(w * i, 0, w, h, QColor('white'), qcolor)

        # Setting marker.
        xpos_marker = (self._xpos_marker * w +
                       (w / 2.0) - (self._timeline_marker_width / 2.0))
        pos_marker = QPointF(xpos_marker, 0)

        # Need to instantiate marker everytime since it gets deleted
        # in every loop by scene.clear()
        timeline_marker = self._instantiate_tl_icon()
        timeline_marker.setPos(pos_marker)
        self._scene.addItem(timeline_marker)

    def _instantiate_tl_icon(self):
        timeline_marker_icon = QIcon.fromTheme('system-search')
        timeline_marker_icon_pixmap = timeline_marker_icon.pixmap(
                                                self._timeline_marker_width,
                                                self._timeline_marker_height)
        return QGraphicsPixmapItem(timeline_marker_icon_pixmap)

    def _get_color_for_value(self, msg):
        """
        :type msg: Gui
        """

        if self._name is not None:
            # look up name in msg; return grey if not found
            info = util.get_info_by_name(msg, self._name)
            if info is not None:
                return util.level_to_color(info.errorlevel) #TODO bullshit !!!
            else:
                return QColor('grey')
        return util.get_color_for_message(msg)
