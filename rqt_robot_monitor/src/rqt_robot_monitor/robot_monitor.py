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
# Author: Isaac Saito, Ze'ev Klapow

import os
import rospkg

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer, Signal
from python_qt_binding.QtGui import QColor
import rospy

from .abst_status_widget import AbstractStatusWidget
from .chronologic_state import InstantaneousState, StatusItem
from .time_pane import TimelinePane
from .util_robot_monitor import Util


class RobotMonitorWidget(AbstractStatusWidget):
    """
    NOTE: RobotMonitorWidget.shutdown function needs to be called
          when the instance of this class terminates.

    RobotMonitorWidget itself doesn't store previous diagnostic states.
    It instead delegates that function to TimelinePane class.
    """

    _sig_tree_nodes_updated = Signal(int)
    _TREE_ALL = 1
    _TREE_WARN = 2
    _TREE_ERR = 3

    def __init__(self, context, topic):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        :param topic: Diagnostic topic to subscribe to ''str''
        """

        super(RobotMonitorWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_robot_monitor'), 'resource',
                               'rqt_robot_monitor_mainwidget.ui')
        loadUi(ui_file, self, {'TimelinePane': TimelinePane})

        obj_name = 'Robot Monitor'
        self.setObjectName(obj_name)
        self.setWindowTitle(obj_name)

        self._toplv_statusitems = []  # StatusItem
        self._warn_statusitems = []  # StatusItem. Contains ALL DEGREES
                                # (device top level, device' _sub) in parallel
        self._err_statusitems = []  # StatusItem

        # This class contains 3 tree widgets.
        # tree_all_devices is all device no matter what the status is.
        # warn/err_flattree shows nodes only when status that fit exist.
        self.tree_all_devices.itemDoubleClicked.connect(self._tree_clicked)
        self.warn_flattree.itemDoubleClicked.connect(self._tree_clicked)
        self.err_flattree.itemDoubleClicked.connect(self._tree_clicked)

        self.tree_all_devices.resizeColumnToContents(0)

        self._sig_tree_nodes_updated.connect(self._tree_nodes_updated)

        self.timeline_pane.set_timeline_data(Util.SECONDS_TIMELINE,
                                          self.get_color_for_value,
                                          self.on_pause)

        self._paused = False
        self._is_stale = False
        self._last_message_time = 0.0

        self._timer = QTimer()
        self._timer.timeout.connect(self._update_message_state)
        self._timer.start(1000)

        self.num_diag_msg_received = 0  # For debug

        self._sub = rospy.Subscriber(
                                    topic,  # name of the topic
                                    DiagnosticArray,  # type of the topic
                                    self._cb)

    def _cb(self, msg):
        """
        Intended to be called from non-Qt thread,
        ie. ROS Subscriber in particular.

        :type msg: DiagnosticArray
        """
        self.new_diagnostic(msg)

    def new_diagnostic(self, msg, is_forced=False):
        """
        Overridden from AbstractStatusWidget.

        When monitoring not paused, this public method updates all the
        treewidgets contained in this class, and also notifies the StatusItem
        instances that are stored in the all-device-tree, which eventually
        updates the InspectorWindows in them.

        :type msg: DiagnosticArray
        :param is_forced: Intended for non-incoming-msg trigger
                          (in particular, from child object like TimelinePane).
        @author: Isaac Saito
        """
        if not self._paused and not is_forced:
            self.timeline_pane.new_diagnostic(msg)

            self._update_devices_tree(msg)
            self._update_flat_tree(msg)

            # timestamp the last msg arrival.
            self._last_message_time = rospy.get_time()

            self._notify_statitems(msg)

            rospy.logdebug('  RobotMonitorWidget _cb stamp=%s',
                       msg.header.stamp)
        elif is_forced:
            self._update_devices_tree(msg)
            self._update_flat_tree(msg)

        self.num_diag_msg_received += 1
        rospy.logdebug('  RobotMonitorWidget _cb #%d',
                       self.num_diag_msg_received)

    def _notify_statitems(self, diag_arr):
        """
        Notify new message arrival to all existing InespectorWindow
        instances that are encapsulated in StatusItem instances contained
        in self._toplv_statusitems.
        """

        for statitem_new in diag_arr.status:
            corresp = Util.get_correspondent(statitem_new.name,
                                             self._toplv_statusitems)
            statitem_prev = corresp[Util.DICTKEY_STATITEM]
            if statitem_prev and statitem_prev.inspector:
                rospy.logdebug('  RobotMonitorWidget _notify_statitems ' +
                               'name=%s len toplv items=%d',
                               statitem_new.name, len(self._toplv_statusitems))
                return

    def resizeEvent(self, evt):
        """
        (Overridden from QWidget.)

        Re-drawn timeline.
        """
        rospy.logdebug('RobotMonitorWidget resizeEvent')
        self.timeline_pane.redraw()

    def _tree_clicked(self, item, column):
        """
        Slot to QTreeWidget.itemDoubleClicked

        :type item: QTreeWidgetItem
        :type column: int
        """
        rospy.logdebug('RobotMonitorWidget _tree_clicked col=%d', column)
        item.on_click()

    def _update_devices_tree(self, diag_array):
        """
        Update the tree that contains all statuses from all devices.

        :type diag_array: DiagnosticArray
        """
        # TODO(Isaac) 11/5/2012 Currently, in case some devices disappear
        #            while running this program, there's no way to remove
        #            those from the device-tree.

        statusnames_curr_toplevel = [Util.get_grn_resource_name(k.name)
                                     for k in self._toplv_statusitems]
        # Only the k variable that pops up at the end is processed
        # by Util.get_grn_resource_name.

        for diagnostic_status_new in (
                    self._get_toplv_diagnosticstatus_from_new_msg(diag_array)):
            name = Util.get_grn_resource_name(diagnostic_status_new.name)
            rospy.logdebug('_update_devices_tree 0 name @ toplevel %s', name)
            dict_status = 0
            if name in statusnames_curr_toplevel:  # No change of names
                                                # in toplevel since last time.
                statusitem = self._toplv_statusitems[
                                        statusnames_curr_toplevel.index(name)]

                dict_status = statusitem.update_children(diagnostic_status_new,
                                                         diag_array)
                times_errors = dict_status[Util.DICTKEY_TIMES_ERROR]
                times_warnings = dict_status[Util.DICTKEY_TIMES_WARN]
                Util.update_status_images(diagnostic_status_new, statusitem)

                #TODO(Isaac) Update status text on each node using dict_status.
                base_text = Util.gen_headline_status_green(statusitem.status)

                rospy.logdebug('_update_devices_tree ' +
                               'diagnostic_status.name = %s\n\t\t\t\t' +
                               'Normal status diag_array = %s',
                               diagnostic_status_new.name, base_text)

                if (times_errors > 0 or times_warnings > 0):
                    base_text = "(Err: %s, Wrn: %s) %s %s" % (
                                   times_errors,
                                   times_warnings,
                                   Util.get_grn_resource_name(
                                                  diagnostic_status_new.name),
                                   diagnostic_status_new.message)
                    rospy.logdebug('_update_dev_tree 1 text to show=%s',
                                   base_text)
                    statusitem.setText(0, base_text)
                    statusitem.setText(1, diagnostic_status_new.message)
                else:
                    rospy.logdebug('_update_dev_tree 2 text to show=%s',
                                   base_text)
                    statusitem.setText(0, base_text)
                    statusitem.setText(1, 'OK')

            else:
                # Create an instance for a category that will be shown at the
                # top level of the tree.
                new_status_item = StatusItem(diagnostic_status_new)

                # Reflect the statuses of all devices under the category.
                new_status_item.update_children(diagnostic_status_new,
                                                diag_array)

                # Figure out if a statusitem and its subtree contains errors.
                # new_status_item.setIcon(0, self._error_icon)
                # This shows NG icon at the beginning of each statusitem.
                Util.update_status_images(diagnostic_status_new,
                                          new_status_item)

                # Add to list of top level items of the tree.
                self._toplv_statusitems.append(new_status_item)

                rospy.logdebug(' _update_devices_tree 2 ' +
                               'diagnostic_status_new.name %s',
                               new_status_item.name)
                # Add to the treewidget.
                self.tree_all_devices.addTopLevelItem(new_status_item)

        # Notify to refresh tree's appearance (currently resizes column size).
        self._sig_tree_nodes_updated.emit(self._TREE_ALL)

    def _tree_nodes_updated(self, tree_type):
        """
        Do tasks to update the tree that corresponds to the given tree_type.
        """
        tree_obj = None
        if self._TREE_ALL == tree_type:
            tree_obj = self.tree_all_devices
        elif self._TREE_WARN == tree_type:
            tree_obj = self.warn_flattree
        if self._TREE_ERR == tree_type:
            tree_obj = self.err_flattree

        # Without calling resizeColumnToContents(0), columns of trees don't
        # expand to adjust the width.
        #
        # Resizing should happen each time tree contents get updated, in order
        # to adjust the width to the maximum length.
        tree_obj.resizeColumnToContents(0)

    def _get_toplv_diagnosticstatus_from_new_msg(self, diag_array):
        """

        Return an array that contains DiagnosticStatus only at the top level of
        the given msg.

        :type msg: DiagnosticArray
        :rtype: DiagnosticStatus[]
        """

        ret = []
        for diagnostic_status in diag_array.status:
            if len(diagnostic_status.name.split('/')) == 2:
                rospy.logdebug(" _get_toplv_diagnosticstatus_from_new_msg " +
                "TOP lev %s ", diagnostic_status.name)
                ret.append(diagnostic_status)
            else:
                rospy.logdebug(" _get_toplv_diagnosticstatus_from_new_msg " +
                               "Not top lev %s ", diagnostic_status.name)
        return ret

    def pause(self, msg):
        """
        Do nothing if already being _paused.

        :type msg: DiagnosticArray
        """
        if not self._paused:
            self._paused = True
            self.new_diagnostic(msg)

    def unpause(self, msg=None):
        """
        :type msg: DiagnosticArray
        """
        self._paused = False

    def on_pause(self, paused, diagnostic_arr):
        """
        Check if InspectorWindows are set. If they are, pause them.

        Pay attention not to confuse with RobotMonitorWidget.pause.

        :type paused: bool
        :type diagnostic_arr: DiagnosticArray
        """

        if paused:
            self.pause(diagnostic_arr)
        elif (len(self._toplv_statusitems) > 0):
            diag_array_queue = self.timeline_pane._get_diagnosticarray()
            statitems = []
            for diag_arr in diag_array_queue:
                state_instant = InstantaneousState()
                state_instant.update(diag_arr)
                statitems.append(state_instant)

        for statitem_toplv in self._toplv_statusitems:
            if (paused):
                statitem_toplv.disable()
            else:
                statitem_toplv.enable()
                for state_instant in statitems:
                    all_states = state_instant.get_items()
                    if statitem_toplv.get_name() in all_states:
                        statitem_toplv.update(
                                all_states[statitem_toplv.get_name()].status)

    def _update_flat_tree(self, diag_arr):
        """
        Update the given flat tree (ie. tree that doesn't show children nodes -
        all of its elements will be shown on top level) with all the
        DiagnosticStatus instances contained in the given DiagnosticArray,
        regardless of the level of the device in a device category.

        For both warn / error trees, StatusItem instances are newly generated.

        :type diag_arr: DiagnosticArray
        """

        for diag_stat_new in diag_arr.status:
            # Num of loops here should be equal to the num of the top
            # DiagnosticStatus item. Ex. in PR2, 9 or so.

            stat_lv_new = diag_stat_new.level
            dev_name = diag_stat_new.name

            # Retrieve previous statusitem instance as dict that matches the
            # key.
            correspondent_warn_curr = Util.get_correspondent(
                                         Util.get_grn_resource_name(dev_name),
                                         self._warn_statusitems)
            correspondent_err_curr = Util.get_correspondent(
                                         Util.get_grn_resource_name(dev_name),
                                         self._err_statusitems)

            # Get index number of the corresponding statusitem instance.
            dev_index_warn_curr = correspondent_warn_curr[Util.DICTKEY_INDEX]
            dev_index_err_curr = correspondent_err_curr[Util.DICTKEY_INDEX]

            # headline is supposed be shown on each category.
            headline = "%s" % diag_stat_new.name

            # If the new status is "OK"
            if DiagnosticStatus.OK == stat_lv_new:
                # if the index of WARN is positive number.
                if 0 <= dev_index_warn_curr:
                    rospy.logdebug('dev_index_warn_curr=%s dev_name=%s, ' +
                                   'stat_lv_new=%d',
                                   dev_index_warn_curr, dev_name, stat_lv_new)
                    # Get the corresponding statusitem from the list of current
                    # WARN statusitems. The copy of the eturned statusitems
                    # still stays in the given list.
                    self._get_statitem(dev_index_warn_curr,
                                                       self._warn_statusitems,
                                                       self.warn_flattree, 1)
                elif 0 <= dev_index_err_curr:
                    self._get_statitem(dev_index_err_curr,
                                                       self._err_statusitems,
                                                       self.err_flattree, 1)

            elif DiagnosticStatus.WARN == stat_lv_new:
                statitem = None
                if 0 <= dev_index_err_curr:
                    # If the corresponding statusitem is in error tree,
                    # move it to warn tree.
                    statitem = self._get_statitem(dev_index_err_curr,
                                                  self._err_statusitems,
                                                  self.err_flattree)
                    self._add_statitem(statitem, self._warn_statusitems,
                                      self.warn_flattree, headline,
                                      diag_stat_new.message, stat_lv_new)
                elif (dev_index_warn_curr < 0 and dev_index_err_curr < 0):
                    # If the corresponding statusitem isn't found,
                    # create new obj.
                    statitem = StatusItem(diag_stat_new)
                    self._add_statitem(statitem, self._warn_statusitems,
                                      self.warn_flattree, headline,
                                      diag_stat_new.message, stat_lv_new)
                    self._warn_statusitems.append(statitem)
                elif (0 < dev_index_warn_curr):
                    # If the corresponding statusitem is already in warn tree,
                    # obtain the instance.
                    statitem = self._get_statitem(dev_index_warn_curr,
                                                  self._warn_statusitems)
                    # Returned statitem instance will be garbage collected.

                #TODO For statitem that is popped and to be dimished, this
                # following step shouldn't be taken.
                if statitem:
                    # Updating statusitem will keep popup window also update.
                    statitem.update_children(diag_stat_new, diag_arr)
            elif ((DiagnosticStatus.ERROR == stat_lv_new) or
                  (DiagnosticStatus.STALE == stat_lv_new)):
                statitem = None
                if 0 <= dev_index_warn_curr:
                    # If the corresponding statusitem is in warn tree,
                    # move it to err tree.
                    statitem = self._get_statitem(dev_index_warn_curr,
                                                  self._warn_statusitems,
                                                  self.warn_flattree)
                    self._add_statitem(statitem, self._err_statusitems,
                                      self.err_flattree, headline,
                                      diag_stat_new.message, stat_lv_new)
                elif (0 <= dev_index_err_curr):
                    # If the corresponding statusitem is already in err tree,
                    # obtain the instance.
                    statitem = self._get_statitem(dev_index_err_curr,
                                                  self._err_statusitems)
                elif (dev_index_warn_curr < 0 and dev_index_err_curr < 0):
                    # If the corresponding statusitem isn't found,
                    # create new obj.
                    statitem = StatusItem(diag_stat_new)
                    self._add_statitem(statitem, self._err_statusitems,
                                      self.err_flattree, headline,
                                      diag_stat_new.message, stat_lv_new)
                if statitem:  # If not None
                    # Updating statusitem will keep popup window also update.
                    statitem.update_children(diag_stat_new, diag_arr)

        self._sig_tree_nodes_updated.emit(self._TREE_WARN)
        self._sig_tree_nodes_updated.emit(self._TREE_ERR)

    def _add_statitem(self, statusitem, statitem_list,
                      tree, headline, statusmsg, statlevel):
        if 'Warning' == statusmsg or 'Error' == statusmsg:
            return

        statusitem.setText(0, headline)
        statusitem.setText(1, statusmsg)
        statusitem.setIcon(0, Util.IMG_DICT[statlevel])
        statitem_list.append(statusitem)
        tree.addTopLevelItem(statusitem)
        rospy.logdebug(' _add_statitem statitem_list length=%d',
                       len(statitem_list))

    def _get_statitem(self, item_index, item_list, tree=None, mode=2):
        """
        Return the item at the corresponding index in the given tree. If mode
        is set appropriately, the item is also removed.

        :type item_list: StatusItem[]
        :param mode: 1 = remove from given list, 2 = w/o removing.
        """
        statitem_existing = item_list[item_index]
        if 1 == mode:
            tree.takeTopLevelItem(tree.indexOfTopLevelItem(statitem_existing))
            item_list.pop(item_index)
        return statitem_existing

    def _update_message_state(self):
        """
        Update the timer counts for the timeline that shows 'last msg was
        received ### secs ago'.
        """
        current_time = rospy.get_time()
        time_diff = current_time - self._last_message_time
        rospy.logdebug('_update_message_state time_diff= %s ' +
                       'self._last_message_time=%s', time_diff,
                       self._last_message_time)
        if (time_diff > 10.0):
            self.timeline_pane._msg_label.setText("Last message received " +
                                               "%s seconds ago"
                                               % (int(time_diff)))
            self._is_stale = True
        else:
            seconds_string = "seconds"
            if (int(time_diff) == 1):
                seconds_string = "second"
            self.timeline_pane._msg_label.setText(
                 "Last message received %s %s ago" % (int(time_diff),
                                                      seconds_string))
            self._is_stale = False

    def shutdown(self):
        """
        This closes all the instances on all trees.
        Also unregisters ROS' subscriber, stops timer.
        Forgetting to call this method whenever this class terminates might
        lead to processes to unexpectedly remain, which could be particularly
        non-trivial when rqt_gui still is running.
        """

        rospy.logdebug('RobotMonitorWidget in shutdown')
        # Close all StatusItem (and each associated InspectWidget)
        # self.tree_all_devices.clear()  # Doesn't work for the purpose
                                        # (inspector windows don't get closed)
        for item in self._err_statusitems:
            item.close()
        for item in self._warn_statusitems:
            item.close()
        for item in self._toplv_statusitems:
            item.close()

        self._sub.unregister()

        self._timer.stop()
        del self._timer

    def save_settings(self, plugin_settings, instance_settings):
        """
        Saves the last status of splitter between status trees.
        """
        instance_settings.set_value('splitter', self.splitter.saveState())

    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.contains('splitter'):
            self.splitter.restoreState(instance_settings.value('splitter'))
        else:
            self.splitter.setSizes([100, 100, 200])

    def _clear(self):
        rospy.logdebug(' RobotMonitorWidget _clear called ')
        self.err_flattree.clear()
        self.warn_flattree.clear()

    def get_color_for_value(self, queue_diagnostic, color_index):
        """
        Overridden from AbstractStatusWidget.

        :type color_index: int
        """

        len_q = len(queue_diagnostic)
        rospy.logdebug(' get_color_for_value color_index=%d len_q=%d',
                      color_index, len_q)
        if (color_index == 1 and len_q == 0):
            return QColor('grey')
        return Util.get_color_for_message(queue_diagnostic[color_index - 1])
                                # When _queue_diagnostic is empty,
                                # this yield error when color_index > 0.
