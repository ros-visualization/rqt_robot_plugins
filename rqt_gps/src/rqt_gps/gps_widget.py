# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Open Source Robotics Foundation (OSRF).
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
# Author: Isaac Saito

import os
import sys

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QModelIndex, Signal
from python_qt_binding.QtGui import QStandardItem, QStandardItemModel, QWidget
import rospkg
import rospy


class GpsWidget(QWidget):
    """#TODO: comment
    """

    # To be connected to PluginContainerWidget
    sig_sysmsg = Signal(str)

    def __init__(self, parent):
        """
        @type parent: GpsMain
        """
        super(GpsWidget, self).__init__()
        self._parent = parent

        self._rospack = rospkg.RosPack()
        ui_file = os.path.join(self._rospack.get_path('rqt_gps'),
                               'resource', 'gps_mainwindow.ui')
        loadUi(ui_file, self)

        self._osm_html_file = os.path.join(self._rospack.get_path('rqt_gps'),
                               'resource', 'osm.html')
        self._set_map_content()

    def _set_map_content(self):
        """
        Set html of OSM
        """
        _htm = self._sample_html_osm()
        self._webview.setHtml(_htm)

    def save_settings(self, plugin_settings, instance_settings):
        # instance_settings.set_value('splitter', self._splitter.saveState())
        pass

    def restore_settings(self, plugin_settings, instance_settings):
#        if instance_settings.contains('splitter'):
#            self._splitter.restoreState(instance_settings.value('splitter'))
#        else:
#            self._splitter.setSizes([100, 100, 200])
        pass

    def _sample_html_osm(self):
        _str_html = ''

        f = open(self._osm_html_file, "r")
        #Read whole file into data
        data = f.read()
        rospy.logdebug(data)
        f.close()
        return data

if __name__ == '__main__':
    # main should be used only for debug purpose.
    # This launches this QWidget as a standalone rqt gui.
    from rqt_gui.main import Main

    main = Main()
    sys.exit(main.main(sys.argv, standalone='rqt_launch'))
