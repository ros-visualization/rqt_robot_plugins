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
# Author: Isaac Saito

import os
import sys

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QModelIndex, Signal
from python_qt_binding.QtGui import QStandardItem, QStandardItemModel, QWidget
from rqt_py_common.data_items import ReadonlyItem

import rospkg
import rospy
from rostopic import get_topic_class, ROSTopicHz
from rqt_topic import topic_info, topic_widget


class MoveitWidget(QWidget):
    """#TODO: comment
    """

    # To be connected to PluginContainerWidget
    sig_sysmsg = None

    def __init__(self, parent, plugin_context):
        """
        @type parent: MoveitMain
        """
        super(MoveitWidget, self).__init__()
        self._parent = parent
        self._plugin_context = plugin_context

        self._rospack = rospkg.RosPack()
        ui_file = os.path.join(self._rospack.get_path('rqt_moveit'),
                               'resource', 'moveit_top.ui')
        loadUi(ui_file, self)

        # topic to show
        self._selected_topics = [('/pointcloud', 'sensor_msgs/PointCloud'),
                                 ('/pointcloud2', 'sensor_msgs/PointCloud2'),
                                 ('/image', 'sensor_msgs/Image'),
                                 ('/camera_info', 'sensor_msgs/CameraInfo')]
        self._widget_topic.set_selected_topics(self._selected_topics)
        self._widget_topic.start()
        # To connect signal in a widget to PluginContainerWidget.
        #TODO: In this way, Signal from only one instance is hooked. 
        # Not a good design at all.
        self.sig_sysmsg = self._widget_topic.sig_sysmsg

        #TODO: Indicate on sys msg pane if capturing these topics fails.
        # That might be because those topics are not published

        self.show()

    def save_settings(self, plugin_settings, instance_settings):
        # instance_settings.set_value('splitter', self._splitter.saveState())
        pass

    def restore_settings(self, plugin_settings, instance_settings):
#        if instance_settings.contains('splitter'):
#            self._splitter.restoreState(instance_settings.value('splitter'))
#        else:
#            self._splitter.setSizes([100, 100, 200])
        pass

    def shutdown(self):
        # TODO: impl
        pass


if __name__ == '__main__':
    # main should be used only for debug purpose.
    # This moveites this QWidget as a standalone rqt gui.
    from rqt_gui.main import Main

    main = Main()
    sys.exit(main.main(sys.argv, standalone='rqt_moveit'))
