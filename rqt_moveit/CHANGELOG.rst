^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.7 (2014-08-18)
------------------

0.3.6 (2014-07-11)
------------------

0.3.5 (2014-06-02)
------------------

0.3.4 (2014-05-07)
------------------

0.3.3 (2014-01-28)
------------------

0.3.2 (2014-01-08)
------------------
* add groups for rqt plugins (`ros-visualization/rqt_common_plugins#167 <https://github.com/ros-visualization/rqt_common_plugins/issues/167>`_)

0.3.1 (2013-10-09)
------------------

0.3.0 (2013-08-28)
------------------

0.2.16 (2013-07-09)
-------------------
* First public release for Hydro
* Fix; Monitoring parameter doesn't turn to "No" even after the params are gone bug (#25 `ros-visualization/rqt_robot_plugins#25 <https://github.com/ros-visualization/rqt_robot_plugins/issues/25>`_)

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
* First public release for Groovy


