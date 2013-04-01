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
    sig_sysmsg = Signal(str)

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

#    def _init_topic_table(self):
#        # name_topic_pointcloud = '/wide_stereo/points'
#        # name_topic_pointcloud2 = '/wide_stereo/points2'
#
#        row = 0
#        for name_topic in self._name_topics:
#            # Create widget for topic.
#            _qitem = ReadonlyItem(name_topic)
#            self._datamodel.setItem(row, 0, _qitem)
#
#            # Create subscriber
#            topic_info = TopicInfo(name_topic)
#            self._topic_obj_dict[name_topic] = topic_info
#
#            row += 1
#
#    def _update_topics_data(self):
#        """
#        Originally copied from rqt_topic.topic_widget._update_topics_data
#        """
#        for topic in self._topic_obj_dict.values():
#            topic_info = topic['info']
#            if topic_info.monitoring:
#                # update rate
#                rate, _, _, _ = topic_info.get_hz()
#                rate_text = '%1.2f' % rate if rate != None else 'unknown'
#
#                # update bandwidth
#                bytes_per_s, _, _, _ = topic_info.get_bw()
#                if bytes_per_s is None:
#                    bandwidth_text = 'unknown'
#                elif bytes_per_s < 1000:
#                    bandwidth_text = '%.2fB/s' % bytes_per_s
#                elif bytes_per_s < 1000000:
#                    bandwidth_text = '%.2fKB/s' % (bytes_per_s / 1000.)
#                else:
#                    bandwidth_text = '%.2fMB/s' % (bytes_per_s / 1000000.)
#
#                # update values
#                value_text = ''
#                self.update_value(topic_info._topic_name,
#                                  topic_info.last_message)
#
#            else:
#                rate_text = ''
#                bandwidth_text = ''
#                value_text = 'not monitored'
#
#            self._topic_obj_dict[topic_info._topic_name].setText(
#                                         self._column_index['rate'], rate_text)
#            self._topic_obj_dict[topic_info._topic_name].setText(
#                               self._column_index['bandwidth'], bandwidth_text)
#            self._topic_obj_dict[topic_info._topic_name].setText(
#                                       self._column_index['value'], value_text)
#
#    def update_topic_table(self):
#        self._update_topics_data()


if __name__ == '__main__':
    # main should be used only for debug purpose.
    # This moveites this QWidget as a standalone rqt gui.
    from rqt_gui.main import Main

    main = Main()
    sys.exit(main.main(sys.argv, standalone='rqt_moveit'))
