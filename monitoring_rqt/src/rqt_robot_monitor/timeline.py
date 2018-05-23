# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Austin Hendrix
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
#  * Neither the name of Austin Hendrix. nor the names of its
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
# Author: Austin Hendrix

import copy
from collections import deque
from python_qt_binding.QtCore import Signal, Slot, QObject

import rospy

from ros_monitoring.msg import *

class Timeline(QObject):
    """
    A class which represents the status history of diagnostics
    It can be queried for a past history of diagnostics, and paused
    """
    message_updated = Signal(Gui)
    queue_updated = Signal(deque)
    pause_changed = Signal(bool)

    def __init__(self, topic, topic_type, count=30):
        super(Timeline, self).__init__()
        self._queue = deque(maxlen=count)
        self._queue_copy = deque(maxlen=count)
        self._count = count
        self._current_index = -1 # rightmost item

        # the paused queue is a backup copy of the queue that is updated with
        # new messages while the timeline is paused, so that new messages and
        # new history are not lost
        self._paused_queue = None

        self._have_messages = False
        self._last_message_time = 0

        self._subscriber = rospy.Subscriber(topic, topic_type, self.callback,
                                            queue_size=10)

    def shutdown(self):
        """
        Turn off this Timeline
        Internally, this just shuts down the subscriber
        """
        self._subscriber.unregister()

    @Slot(bool)
    def set_paused(self, pause):
        """
        Slot, to be called to change the pause status of the timeline

        This is generally intended to be connected to the status signal
        from a button or checkbox
        """
        if pause != self.paused:
            if pause:
                self._paused_queue = deque(self._queue, self._queue.maxlen)
            else:
                self._queue = self._paused_queue
                self._queue_copy = copy.deepcopy(self._paused_queue)
                self._paused_queue = None

                # update pointer to latest message
                self._current_index = -1
                self.message_updated.emit(self._queue[self._current_index])
            self.pause_changed.emit(pause)

    @property
    def paused(self):
        """ True if this timeline is paused """
        return self._paused_queue is not None

    def callback(self, msg):
        """
        ROS Callback for new diagnostic messages

        Puts new msg into the queue, and emits a signal to let listeners know
        that the timeline has been updated

        If the timeline is paused, new messages are placed into a separate
        queue and swapped back in when the timeline is unpaused

        :type msg: Either Gui or DiagnosticsStatus. Can be
                   determined by __init__'s arg "msg_callback".
        """
        self._have_messages = True
        self._last_message_time = rospy.get_time()
        if self.paused:
            self._paused_queue.append(msg)
        else:
            self._queue.append(msg)
            self._queue_copy = copy.deepcopy(self._queue)
            self.queue_updated.emit(self._queue_copy)
            self.message_updated.emit(msg)

    @property
    def queue(self):
        return self._queue_copy

    @property
    def has_messages(self):
        """
        True if this timeline has received any messages.
        False if no messages have been received yet
        """
        return self._have_messages

    def data_age(self):
        """ Get the age (in seconds) of the most recent diagnostic message """
        current_time = rospy.get_time()
        time_diff = current_time - self._last_message_time
        return time_diff

    @property
    def is_stale(self):
        """ True is this timeline is stale. """
        return self.data_age() > 10.0

    def set_position(self, index):
        max_index = len(self._queue) - 1
        min_index = -len(self._queue)
        index = min(index, max_index)
        index = max(index, min_index)
        if index != self._current_index:
            self._current_index = index
            self.message_updated.emit(self._queue[index])

    def get_position(self):
        index = self._current_index
        if index < 0:
            index = len(self._queue) + index
        return index

    def __len__(self):
        return len(self._queue)

    def __iter__(self):
        for msg in self._queue:
            yield msg
