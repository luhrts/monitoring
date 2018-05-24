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
#
# TODO(ahendrix):
#   better formatting of key-value pairs in a table

from python_qt_binding.QtWidgets import QTextEdit
from python_qt_binding.QtCore import Signal

from diagnostic_msgs.msg import DiagnosticStatus
from rqt_robot_monitor.util_robot_monitor import level_to_text

from monitoring_msgs.msg import *


class StatusSnapshot(QTextEdit):
    """Display a single static status message. Helps facilitate copy/paste"""
    write_status = Signal(GuiInfo)

    def __init__(self, status=None, parent=None):
        super(StatusSnapshot, self).__init__()

        self.write_status.connect(self._write_status)
        if status is not None:
            self.write_status.emit(status)

            self.resize(300, 400)
            self.show()

    def _write_status(self, status):
        self.clear()
        self._write("Full Name", status.name)
        self._write("Component", status.name.split('/')[-1])
#         self._write("Hardware ID", status.hardware_id)
        self._write("Level", level_to_text(status.errorlevel))
#         self._write("Message", status.message)
        self.insertPlainText('\n')

#         for value in status.infos: #TODO
        self._write(status.name, status.value)

    def _write(self, k, v):
        # TODO(ahendrix): write these as a table rather than as text
        self.setFontWeight(75)
        self.insertPlainText(str(k))
        # TODO(ahendrix): de-dupe trailing ':' here
        self.insertPlainText(': ')

        self.setFontWeight(50)
        self.insertPlainText(str(v))
        self.insertPlainText('\n')
