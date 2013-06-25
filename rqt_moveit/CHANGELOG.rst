^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix to Issue #25: Monitoring parameter doesn't turn to "No" even after the params are gone bug 
* Refactoring
  * Removed unnecessary dependency (ie. rqt_moveit doesn't depend on MoveIt!)
  * Common steps ported to rqt_py_common
  * Add exception handling around rosnode_ping.
* Documentation
  * Notation corrected (MoveIt! is correct)
  * Description improved in package.xml

0.2.15 (2013-04-25)
-------------------
* Fix
  * monitoring parameter wasn't working
  * Removed dependency to MoveIt! `#20 <https://github.com/rqt_robot_plugins/rqt_robot_plugins/issues/20>`
  * segfaults when plugin shutdown bug `#14 <https://github.com/rqt_robot_plugins/rqt_robot_plugins/issues/14>`
* Enhancement
  * Change refresh rate enhancement `#15 <https://github.com/rqt_robot_plugins/rqt_robot_plugins/issues/15>`
  * more efficient GUI layout
  * GUI minor improve (adjust width of tableview to its contents length)
  * Add clarification to its name displayed on rqt_gui (rqt_moveit only monitors, not interact with Moveit!).
  * Updated plugin desc displayed.
* Refactoring
  * exception handling added
  * Topic pane is added to QSplitter (a little hacky, but Custom widget classes don't show in QSplitter when they instantiated in .ui file and not explicitly added to QSplitter. Thus this submits only a workaround. Perm solution is ideal)
  * Removed unnecessary files

0.2.14 (2013-04-12)
-------------------

0.2.13 (2013-04-09)
-------------------
* correct wrong dependency signature.
* initial commit.

