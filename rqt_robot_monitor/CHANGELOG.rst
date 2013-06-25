^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_robot_monitor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.2.15 (2013-04-25)
-------------------
* 0.2.15
* Many pkgs) Corret and clean up dependencies.
* rqt_robot_monitor) Correct missing/unnecessary dependency.
* 0.2.14

0.2.14 (2013-04-12)
-------------------

0.2.13 (2013-04-09)
-------------------

0.2.12 (2013-04-06 18:22)
-------------------------

0.2.11 (2013-04-06 18:00)
-------------------------

0.2.10 (2013-04-04)
-------------------
* rqt_robot_monitor) another debug to issue `#13 <https://github.com/130s/rqt_robot_plugins/issues/13>`_.
* rqt_robot_monitor) minor refactoring, cleaning.
* rqt_robot_monitor) fix to Issue `#13 <https://github.com/130s/rqt_robot_plugins/issues/13>`_.
* rqt_robot_monitor) modified description
* rqt_robot_monitor) authors added.
* rqt_robot_monitor) Modify email address

0.2.9 (2013-03-07)
------------------
* rqt_robot_monitor) Debug (now run with pyside. It used to be not working with it w/o noticing...)
* rqt_robot_monitor) Bug fixed. Call .ui file in .ui is now successfully working too.
* rqt_robot_monitor) Fixed bugs that were caused somehow in cleaning process in the last few commits.
* rqt_robot_monitor: Now uses UI file for TimelinePane
* rqt_robot_monitor: pep8 changes
* rqt_robot_monitor) Refactoring.
* rqt_robot_monitor) Complete PEP8 adjustment.
* rqt_robot_monitor: some pep8 changes
* rqt_robot_monitor) minor edit (add description to package.xml)
* rqt_robot_monitor) Add docroot to public methods.
* rqt_robot_monitor) Minor (code repo url specified more)

0.2.8 (2013-01-11)
------------------
* fix urls in package.xml files

0.2.7 (2012-12-23 15:58)
------------------------

0.2.6 (2012-12-23 01:57)
------------------------

0.2.5 (2012-12-21 19:11)
------------------------
* adding missing license headers

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
* catkinize shell scripts and removed load_manifest calls
* catkinizing
  catkinize: remaining uncatkinized packages
  catkinize: removed rqt stack
  catkinize: removing Makefiles
  catkinize: added version tags
  catkinize: fixed version numbers
  catkinize: added setup.py files
  catkinize: added setup.py files
  catkinize: fixed cmake lists
  catkinize: removed rosdeps
  catkinize: building
  catkinize: added setup.py files
  catkinize: move plugin.xmls
  catkinize: add qt_gui depend
  catkinize: added repositories
  fix deps
  add .gitignore
  fix export tag in rqt_gui_py
  add missing target_link_libraries
  catkinize: rqt_msg, rqt_bag, rqt_console, rqt_bag_plugins now work in install space
  catkinize: all python plugins run from their scripts in install space
  rqt_nav_view: Removed extra plugin install rule.
  disable graying out unavailable python plugins
  catkinize: c++ plugins working, rosruning working in install/build spaces
  catkinize: removed rosmake style .gitignore files
  catkinize: fixed the "s"
  add missing inc dirs
* rqt_robot_monitor) Fix to Issue `#49 <https://github.com/130s/rqt_robot_plugins/issues/49>`_.
* rqt_robot_monitor) Code cleaned (docstring formatted).
* rqt_robot_monitor) Code cleaned (thread-unsafe callback removed, unused codes removed, a file with multiple classes that don't make much sense is splitted docstring is adjusted to PEP257 + Sphinx format).

0.1.7 (2012-12-13 16:17)
------------------------

0.1.6 (2012-12-13 14:43:47)
---------------------------
* rqt_robot_monitor, rqt_robot_dashboard) (1) API (func names) changed for shutdown procedure, (2) funcs & variables names update (pub & private were wrongly declared before), (3) a few issues on the timeline on popup windows fixed (updated twice at a second, old values not available).
* rqt_robot_monitor) trivial) Undesirable callback hook is corrected.

0.1.5 (2012-12-08)
------------------
* rqt_robot_monitor) bugfixed (timeline doesn't adjuts its size when paused).
* rqt: removing incorrect entries in gitignore files
* rqt_robot_monitor) Added guide text on .ui file (about timeline's direction)
* rqt_robot_monitor/_dashboard) API name changed (conformed to PEP8).
* rqt_robot_monitor) Bug fixed (timeline often shows blank cell on the right end).
* rqt_robot_monitor) trivial) Revert wrong function name operation.
* rqt_robot_monitor) Trivial) Redundant "self" is removed.
* rqt_robot_monitor) Added new feature: introspection by timeline. Also merged with the latest push.
* rqt_pr2_dashboard: implemented save/restore of subwidget states
* rqt: conversion of scripts to new qt_gui standalone package argument system
* rqt: fix for standalone scripts to allow for standalone arguments
* rqt_robot_monitor) Fix to Issue `#42 <https://github.com/130s/rqt_robot_plugins/issues/42>`_.
* rqt_robot_monitor: style + remove close in the shutdown function
* rqt_robot_monitor: fix for timeline resizing issue
* rqt_robot_monitor: removed click fucntion from timeline for 1.5 release. Added bool param for pause click
* rqt: script overhaul for running plugins in standalone mode
* rqt_robot_monitor) Fix to Issue `#37 <https://github.com/130s/rqt_robot_plugins/issues/37>`_.
* rqt_robot_monitor) Adding 2 new .py files (missed to be added in 79271926df0fe45fcb5fba317291c00e8c56b93a)
* rqt_robot_monitor) Split a file into 2 files to avoid confusion. Thus adding 2 new files and removing the old file.
* rqt_robot_monitor) Fix to issue`#32 <https://github.com/130s/rqt_robot_plugins/issues/32>`_ (not displaying icons on some computers). Switched to the icons that should be more common.
* rqt_robot_monitor) Fixed icon for the status STALE not appearing on Error pane.
* rqt_robot_monitor: Modified a feature; Warn & Error trees now show only the most detail (ie. abstracted warn/error content no longer shown) to provide quicker access to the specific detail.
* rqt_robot_monitor: 2 major bugs fixed. (1) causes rqt_gui to segfaults when more Error/Warn signals are received (5+ when I observed), (2) Thread alive even after shutting down the plugin which sometimes end up error close.
* rqt_robot_monitor: Added a feature (time elapsed after the last topic/msg received is now shown).

0.1.4 (2012-11-20)
------------------

0.1.3 (2012-11-19)
------------------
* rqt_robot_monitor: Feature added (columns' width in trees auto-adjusted to its content length).
* rqt_robot_monitor: Avoid error print (that happens occasionally esp. when used via PR2 Dashboard)
  <quote>
  [ERROR] [WallTime: 1353027086.052926] bad callback: <bound method RobotMonitorWidget._cb of <rqt_robot_monitor.robot_monitor.RobotMonitorWidget object at 0x36e2050>>
  Traceback (most recent call last):
  File "/opt/ros/fuerte/lib/python2.7/dist-packages/rospy/topics.py", line 678, in _invoke_callback
  cb(msg)
  File "/u/ablasdel/ros_sandbox/ros_gui/rqt/rqt_robot_monitor/src/rqt_robot_monitor/robot_monitor.py", line 302, in _cb
  self._update_warns_errors(msg)
  File "/u/ablasdel/ros_sandbox/ros_gui/rqt/rqt_robot_monitor/src/rqt_robot_monitor/robot_monitor.py", line 451, in _update_warns_errors
  self._update_flat_tree(diag_array)
  File "/u/ablasdel/ros_sandbox/ros_gui/rqt/rqt_robot_monitor/src/rqt_robot_monitor/robot_monitor.py", line 659, in _update_flat_tree
  itemtree.addTopLevelItem(statitem_new)
  AttributeError: 'NoneType' object has no attribute 'addTopLevelItem'
  </quote>
* rqt_bag/console/robot_monitor: added customwidgets to fix pyside failure to override default event handlers
* rqt_robot_monitor: Bug fixed (variable names messed up while cleaning code in the previous commits).
* rqt_robot_monitor: The rest of 3 .py files conformed to PEP8.
* rqt_robot_monitor: Code cleaned & conformed to PEP8 (as much as possible) in robot_monitor.py
* rqt_robot_monitor: Code cleaned a little (fixed functions publicly accessed having "_" at the beginning of their name)
  Conflicts:
  rqt_robot_monitor/src/rqt_robot_monitor/robot_monitor.py
* rqt_robot_monitor: A function renamed (_shutdown --> shutdown, after it turned out to be accessed publicly).
* rqt_robot_monitor: Code cleaned a little (fixed functions publicly accessed having "_" at the beginning of their name)
* rqt_robot_monitor: 2 following bugs fixed.
  - A warning item never gets removed on Warning tree even after the warning is gone.
  - rospy.Subscriber thread persists even after the plugin is closed (by either cross button or action menu)
* rqt_robot_monitor: 2 following bug fixed.
  - A warning item never gets removed on Warning tree even after the warning is gone.
  - rospy.Subscriber thread persists even after the plugin is closed (by either cross button or action menu)
* rqt_robot_monitor: Fix to https://github.com/ros-visualization/rqt/issues/11
  Removed feature: Sorting each tree.
* rqt_robot_monitor:
  Issue fixed:
  - Wrong reference is removed (to a class with old name)
* rqt_various: added or fixed support for appending iteration numbers (%d) to the titles to easily identify multiple plugin instances
* rqt_robot_monitor:
  Added functionality:
  - Trees separate columns for showing device names and their message content.
* rqt_robot_monitor:
  Fixing defects / unsuitable behaviors:
  - Pop-up windows never shows after they're opened for the 1st time.
  - Pop-up windows remain open even rqt_gui is closed.
  - Size of each tree panes are fixed and never be able to be resized.
  - Scroll bar's position gets reset per second.
  Adding functionalities:
  - Show warn/error msg content on tree nodes
  - Show icons that stand for the device status
  Adding design features:
  - Introduce .ui file (ie. removing layout description from source code as much as possible).
* rqt_nav_view,rqt_robot_dashboard,rqt_robot_monitor: Fixed the "old catkin version" and put these back to the previous system to be properly catkinized in the future
* rqt_nav_view/rqt_robot_monitor: fixed explicit use of pyqtsignal
* rqt_robot_monitor/rqt_nav_view: modified to allow them to run properly as plugins after the move to rqt and renaming
* rqt_nav_view,rqt_robot_dashboard,rqt_robot_monitor: deleted stack.xml so they are part of the rqt stack (will catkinize later)
* rqt_pr2_dashboard and merged dependancies: Fixed instances of naming problems in code to allow rqt_pr2_dashboard to run again
* rqt_robot_monitor: initial import
  Split robot_monitor from robot_dahsboard.
  Robot monitor should export its own plugin.
  Fix catkin python setup.
  Fix not closing properly.
  Status should actually update.
  Update src/robot_monitor/robot_monitor.py
  Added window title
  Timeline displays. Not yet clickable.
  Timeline is now clickable.
  Double timeline queue size.
  Only valid messages are clickable.
  Inspectors now have timelines.
  Make sure levels are always correct.
  Fix layout and sizing.
  Preparing for import into rqt
