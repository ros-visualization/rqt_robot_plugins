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
import threading

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QModelIndex, QTimer, Signal
from python_qt_binding.QtGui import QStandardItem, QStandardItemModel, QWidget
import rospkg
from rosnode import rosnode_ping
import rospy
from rqt_topic.topic_widget import TopicWidget


class NodeMonitorThread(threading.Thread):
    def __init__(self, parent, signal_tobe_emitted, nodes_monitored):
        """
        @type nodes_monitored: str[]
        """
        super(NodeMonitorThread, self).__init__()
        self._parent = parent
        self._signal = signal_tobe_emitted
        self._nodes_monitored = nodes_monitored

        self._timer = QTimer()
        self._timer.timeout.connect(self._monitor_nodes)

    def run(self):
        self._timer.start(500)

    def _monitor_nodes(self):
        for nodename in self._nodes_monitored:
            is_node_running = rosnode_ping(nodename, 1)

            #TODO: segfault occurs here most of every time the plugin shut down
            self._signal.emit(is_node_running, nodename)


class ParamCheckThread(threading.Thread):
    def __init__(self, parent, signal_tobe_emitted, params_monitored):
        """
        @type params_monitored: str[]
        """
        super(ParamCheckThread, self).__init__()
        self._parent = parent
        self._signal = signal_tobe_emitted
        self._params_monitored = params_monitored

        self._timer = QTimer()
        self._timer.timeout.connect(self._monitor_param)

    def run(self):
        self._timer.start(300)

    def _monitor_param(self):
        for param in self._params_monitored:
            has_param = rospy.has_param(param)
            self._signal.emit(has_param, param)


class MoveitWidget(QWidget):
    """#TODO: comment
    """

    # To be connected to PluginContainerWidget
    sig_sysmsg = None
    sig_param = Signal(bool, str)  # param name emitted
    sig_node = Signal(bool, str)  # param name emitted

    def __init__(self, parent, plugin_context):
        """
        @type parent: MoveitMain
        """

        self._nodes_monitored = ['/move_group']
        self._selected_topics = [('/pointcloud', 'sensor_msgs/PointCloud'),
                                 ('/pointcloud2', 'sensor_msgs/PointCloud2'),
                                 ('/image', 'sensor_msgs/Image'),
                                 ('/camera_info', 'sensor_msgs/CameraInfo')]
        _params_monitored = ['/robot_description',
                             '/robot_description_semantic']

        super(MoveitWidget, self).__init__()
        self._parent = parent
        self._plugin_context = plugin_context

        self._rospack = rospkg.RosPack()
        ui_file = os.path.join(self._rospack.get_path('rqt_moveit'),
                               'resource', 'moveit_top.ui')
        loadUi(ui_file, self, {'TopicWidget': TopicWidget})

        # Monitor node
        self._node_qitems = {}
        self._init_monitor_nodes(self._nodes_monitored)
        #TODO: connect sys msg for nodes.

        # topic to show
        # Delegate GUI functionality to rqt_topic.TopicWidget.
        self._widget_topic.set_selected_topics(self._selected_topics)
        self._widget_topic.set_topic_specifier(TopicWidget.SELECT_BY_MSGTYPE)
        self._widget_topic.start()
        # To connect signal in a widget to PluginContainerWidget.
        #TODO: In this way, Signal from only one instance is hooked.
        # Not a good design at all.
        self.sig_sysmsg = self._widget_topic.sig_sysmsg

        # Init monitoring parameters.
        self._param_qitems = {}
        self._init_monitor_parameters(_params_monitored)

    def _init_monitor_nodes(self, nodes_monitored):
        """
        @type params_monitored: str[]
        """
        self._node_datamodel = QStandardItemModel(0, 2)
        self._root_qitem = self._node_datamodel.invisibleRootItem()
        self._view_nodes.setModel(self._node_datamodel)

        self._node_monitor_thread = NodeMonitorThread(self, self.sig_node,
                                                      nodes_monitored)
        self.sig_node.connect(self._monitor_nodes)
        self._node_monitor_thread.start()

    def _init_monitor_parameters(self, params_monitored):
        """
        @type params_monitored: str[]
        """
        self._param_datamodel = QStandardItemModel(0, 2)
        self._root_qitem = self._param_datamodel.invisibleRootItem()
        self._view_reconf.setModel(self._param_datamodel)

        self._param_check_thread = ParamCheckThread(self, self.sig_param,
                                                    params_monitored)
        self.sig_param.connect(self._monitor_parameters)
        self._param_check_thread.start()

    def _monitor_nodes(self, is_node_running, node_name):
        """
        Slot

        @type is_node_running: bool
        @type node_name: str
        """
        #TODO: What this method does is exactly the same with
        # monitor_parameters. Unify them.

        rospy.logdebug('is_node_running={} par_name={}'.format(is_node_running,
                                                              node_name))
        node_name = str(node_name)
        node_qitem = None
        if not node_name in self._node_qitems:
            node_qitem = QStandardItem(node_name)
            self._node_qitems[node_name] = node_qitem
            self._node_datamodel.appendRow(node_qitem)
        else:  # qsitem with the node name already exists.
            node_qitem = self._node_qitems[str(node_name)]

        qindex = self._node_datamodel.indexFromItem(node_qitem)
        _str_node_running = 'Not running'
        if is_node_running:
            _str_node_running = 'Running'
        qitem_node_status = QStandardItem(_str_node_running)
        self._node_datamodel.setItem(qindex.row(), 1, qitem_node_status)

    def _monitor_parameters(self, has_param, param_name):
        """
        Slot

        @type has_param: bool
        @type param_name: str
        """
        rospy.logdebug('has_param={} par_name={}'.format(has_param, param_name))
        param_name = str(param_name)
        param_qitem = None
        if not param_name in self._param_qitems:
            param_qitem = QStandardItem(param_name)
            self._param_qitems[param_name] = param_qitem
            self._param_datamodel.appendRow(param_qitem)
        else:  # qsitem with the param name already exists.
            param_qitem = self._param_qitems[str(param_name)]

        qindex = self._param_datamodel.indexFromItem(param_qitem)
        _str_param_found = 'Not found on Parameter Server'
        if has_param:
            _str_param_found = 'Found on Parameter Server'
        qitem_param_status = QStandardItem(_str_param_found)
        self._param_datamodel.setItem(qindex.row(), 1, qitem_param_status)
        #.insertColumn([qitem_param_status])

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
