^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix to Issue #25: Monitoring parameter doesn't turn to "No" even after the params are gone bug (`ros-visualization/rqt_robot_plugins#25 <https://github.com/ros-visualization/rqt_robot_plugins/issues/25>`_)

* Refactoring

  * Removed unnecessary dependency (ie. rqt_moveit doesn't depend on any of MoveIt! packages)
  * Common steps ported to rqt_py_common pkg
  * Add exception handling around rosnode_ping

* Notation corrected (MoveIt! is correct) @davetcoleman

0.2.15 (2013-04-25)
-------------------
* Fix

  * monitoring parameter wasn't working
  * Removed dependency to MoveIt! (`#20 <https://github.com/rqt_robot_plugins/rqt_robot_plugins/issues/20>`_)
  * segfaults when plugin shutdown bug (`#14 <https://github.com/rqt_robot_plugins/rqt_robot_plugins/issues/14>`_)

* Enhancement

  * Change refresh rate enhancement (`#15 <https://github.com/rqt_robot_plugins/rqt_robot_plugins/issues/15>`_)
  * More efficient GUI layout (packed empty regions)
  * Layout improve (adjust width of tableview to its contents length)
  * Add clarification to its name displayed on rqt_gui (rqt_moveit only monitors, not interact with Moveit!).

0.2.14 (2013-04-12)
-------------------

0.2.13 (2013-04-09)
-------------------
* first public release for Groovy


