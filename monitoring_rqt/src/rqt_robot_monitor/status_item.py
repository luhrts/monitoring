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

from python_qt_binding.QtWidgets import QTreeWidgetItem
import rqt_robot_monitor.util_robot_monitor as util

from ros_monitoring.msg import *

class _StatusItem(QTreeWidgetItem):
    """
    Internal subclass of QTreeWidgetItem which adds a 'name' member to make
    it easier to extract the item name and create an inspector when an item
    is clicked
    """
    def __init__(self, name):
        super(_StatusItem, self).__init__()
        self.name = name

class StatusItem(object):
    """
    A class that wraps the default QTreeWidgetItem, so that we can manipulate
    all of the nodes in the tree in the same way (even the invisible root node)
    """
    def __init__(self, item=None):
        self._children = {}
        self.updated = False
        if item is not None:
            self._item = item
        else:
            self._item = _StatusItem("NONAME")

    def update(self, info, displayname):
        self.updated = True
        self.displayname = displayname
        self._item.name = info.name
        self._item.setText(0, self.displayname)
        self._item.setIcon(0, util.level_to_icon(info.errorlevel)) 
        self._item.setText(1, info.value + " " + info.unit)
        self._item.setText(2, str(info.errorlevel)) #TODO
            

    def prune(self):
        stale = []
        for child in self._children:
            if not self._children[child].updated:
                stale.append(child)
            else:
                self._children[child].prune()
        if len(stale) > 0:
            for child in stale:
                self._item.removeChild(self._children[child]._item)
                del self._children[child]
        self.updated = False

    # functions that make this object behave like a dict
    def __getitem__(self, key):
        # if item doesn't exist, create a sane default for it
        if not key in self._children:
            self._children[key] = StatusItem()
            self._item.addChild(self._children[key]._item)
        return self._children[key]

    def __setitem__(self, key, value):
        # if item exists, remove it from the tree in addition to overwriting
        # it in the 'children' dict
        if key in self._children:
            self._item.removeChild(self._children[key]._item)
        self._children[key] = value
        self._item.addChild(value._item)

    def __contains__(self, key):
        return key in self._children

    def __iter__(self):
        for key in self._children:
            yield key
