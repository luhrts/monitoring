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

from python_qt_binding.QtGui import QColor, QIcon
import rospy

# TODO: Utils and common configs are mixed in this class.

# Instantiating icons that show the device status.
_ERR_ICON = QIcon.fromTheme('dialog-error')
_WARN_ICON = QIcon.fromTheme('dialog-warning')
_OK_ICON = QIcon.fromTheme('emblem-default')
# Added following this QA thread http://goo.gl/83tVZ
_STALE_ICON = QIcon.fromTheme('dialog-question')

_LEVEL_TO_ICON = {0: _OK_ICON, 1: _WARN_ICON, 2: _ERR_ICON, 3: _STALE_ICON}

_LEVEL_TO_COLOR = {0: QColor(85, 178, 76),  # green
                   1: QColor(222, 213, 17), # yellow
                   2: QColor(178, 23, 46),  # red
                   3: QColor(40, 23, 176)   # blue
                   }

_LEVEL_TO_TEXT = { 0: "OK", 1: "WARNING", 2: "ERROR", 3: "STALE" }

def level_to_icon(errorlevel):
    level = int(errorlevel*2.9)
    if level in _LEVEL_TO_ICON:
        return _LEVEL_TO_ICON[level]
    else:
        return _ERR_ICON

def level_to_color(errorlevel):
    level = int(errorlevel*2.9)
    if level in _LEVEL_TO_COLOR:
        return _LEVEL_TO_COLOR[level]
    else:
        return _LEVEL_TO_COLOR[2]

def level_to_text(errorlevel):
    level = int(errorlevel*2.9)
    if level in _LEVEL_TO_TEXT:
        return _LEVEL_TO_TEXT[level]
    else:
        return "UNKNOWN(%d)" % ( level )

def get_resource_name(status_name):
    """
    Get resource name from path

    :param: status_name is a string that may consists of status names that
            are delimited by slash.
    :rtype: str
    """
    name = status_name.split('/')[-1]
    rospy.logdebug(' get_resource_name name = %s', name)
    return name

def get_color_for_message(msg):
    """
    Get the overall (worst) color for a Gui
    :param msg: Gui
    """
    level = 0.0
    min_level = 1.0

    lookup = {}
    
    if (msg.errorlevel > level):
        level = msg.errorlevel
    if (msg.errorlevel < min_level):
        min_level = msg.errorlevel

    # Stale items should be reported as errors unless all stale
    if (level > 1.0 and min_level <= 1.0):
        level = 1.0

    rospy.logdebug(' get_color_for_message color lv=%d', level)
    return level_to_color(level)


def get_info_by_name(msg, name):
    for info in msg.infos:
        if info.name == name:
            return info
    return None
