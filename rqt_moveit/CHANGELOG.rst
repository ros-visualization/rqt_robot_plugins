^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* MoveIt! has a capital I :)
* minor change to pkg description
* minor change to pkg description.
* port common set of steps to a new method in rqt_py_common.
* dependency updated.
* Fix to Issue `#25 <https://github.com/130s/rqt_robot_plugins/issues/25>`_
* Still adding pkg description.
* description added.
* add exception handling around rosnode_ping.

0.2.15 (2013-04-25)
-------------------
* rqt_pose_view) fix dependency corrupted by recent modification.
  Revert "0.2.15", due to wrong dependency in rqt_pose_view.
  This reverts commit 021213f7fa3d73ab7a09ee45603249da2820d17a.
  0.2.15
* GUI layout improvement.
* minor improve (exception handling added)
* minor cleaning, refactoring
* Debug (monitoring parameter wasn't working).
* Revert "Debug (monitoring parameter wasn't working)"
  This reverts commit 5504d2d9eb01d96591da05da064cf21f9d9480f5.
* Debug (monitoring parameter wasn't working)
* Enhancement based on Issue `#15 <https://github.com/130s/rqt_robot_plugins/issues/15>`_.
* 0.2.14
* Following descussion in Issue `#20 <https://github.com/130s/rqt_robot_plugins/issues/20>`_.
* Topic pane is added to QSplitter (a little hacky, but Custom widget classes don't show in QSplitter when they instantiated in .ui file and not explicitly added to QSplitter. Thus this submits only a workaround. Perm solution is ideal)
* Updated plugin desc displayed.
* Add clarification to its name displayed on rqt_gui (rqt_moveit only monitors, not interact with Moveit!).
* GUI minor improve (adjust width of tableview to its contents length)
* fix to Issue `#14 <https://github.com/130s/rqt_robot_plugins/issues/14>`_
* Removed an unnecessary file

0.2.14 (2013-04-12)
-------------------

0.2.13 (2013-04-09)
-------------------
* correct wrong dependency signature.
* initial commit.

0.2.12 (2013-04-06 18:22)
-------------------------

0.2.11 (2013-04-06 18:00)
-------------------------
* Remove rqt_moveit & rqt_marble since the development of them are not matured enough and move to @130s private repos.

0.2.10 (2013-04-04)
-------------------
* minor ui improvement (introduce splitter. But TopicWidget cannot be shown if it's included in QSplitter. Needs investigated)
* Minor update (status msg improved)
* Add topic monitoring feature (based on rqt_topic). With this commit, full very basic features become available although still unstable.
* add node monitoring/pinging feature.
* add parameter monitoring feature.
* Added param update (not working well)
* Exception handled for Topic pane.
* initial commit. (by accident rqt_marble's minor cleaning gets included too).

0.2.9 (2013-03-07)
------------------

0.2.8 (2013-01-11)
------------------

0.2.7 (2012-12-23 15:58)
------------------------

0.2.6 (2012-12-23 01:57)
------------------------

0.2.5 (2012-12-21 19:11)
------------------------

0.2.4 (2012-12-21 01:13)
------------------------

0.2.3 (2012-12-21 00:24)
------------------------

0.2.2 (2012-12-20 18:29)
------------------------

0.2.1 (2012-12-20 17:47)
------------------------

0.2.0 (2012-12-20 17:39)
------------------------

0.1.7 (2012-12-13 16:17)
------------------------

0.1.6 (2012-12-13 14:43:47)
---------------------------

0.1.5 (2012-12-08)
------------------

0.1.4 (2012-11-20)
------------------

0.1.3 (2012-11-19)
------------------
