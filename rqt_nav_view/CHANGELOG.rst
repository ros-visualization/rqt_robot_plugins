^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_nav_view
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* rqt_nav_view) Fix to Issue `#27 <https://github.com/130s/rqt_robot_plugins/issues/27>`_

0.2.15 (2013-04-25)
-------------------
* 0.2.15
* Many pkgs) Corret and clean up dependencies.
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
* maintainer added to several pkgs

0.2.9 (2013-03-07)
------------------
* rqt_robot_dashboard,nav_view: ensured that nav_view_dash_widget removes the widget when clicked a second time

0.2.8 (2013-01-11)
------------------
* fix urls in package.xml files
* rqt_nav_view: Map scrolling fixed.
* rqt_nav_view: NavFn should be the default.
* rqt_nav_view: Goals can now be set.
* rqt_nav_view: Drag and drop works everywhere now.
* rqt_nav_view: Initial pose can now be set.
* rqt_nav_view: Don't update without map.
* rqt_nav_view: Don't regenerate unless map changes.
* rqt_nav_view: Only use one tranform listener.
* rqt_nav_view: Topics can be drag and dropped.
* rqt_nav_view: Fix map colors.

0.2.7 (2012-12-23 15:58)
------------------------

0.2.6 (2012-12-23 01:57)
------------------------

0.2.5 (2012-12-21 19:11)
------------------------
* pep8 style changes
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

0.1.7 (2012-12-13 16:17)
------------------------

0.1.6 (2012-12-13 14:43:47)
---------------------------

0.1.5 (2012-12-08)
------------------
* rqt: removing incorrect entries in gitignore files
* rqt_nav_view: moved from scaling zoom to viewscaling zoom
* rqt_nav_view: port to use native QImages instead of PIL images due to PIL being PyQt only
* rqt: conversion of scripts to new qt_gui standalone package argument system
* rqt: fix for standalone scripts to allow for standalone arguments
* rqt: script overhaul for running plugins in standalone mode
* rqt_nav_view: Added Exception and explaination of the limitation of one plugin per rqt instance
* rqt_nav_view: fixed failure to unsubscribe to topics

0.1.4 (2012-11-20)
------------------

0.1.3 (2012-11-19)
------------------
* rqt_various: fixed PySide specific errors
* rqt_various: added or fixed support for appending iteration numbers (%d) to the titles to easily identify multiple plugin instances
* rqt_nav_view,rqt_robot_dashboard,rqt_robot_monitor: Fixed the "old catkin version" and put these back to the previous system to be properly catkinized in the future
* rqt_nav_view/rqt_robot_monitor: fixed explicit use of pyqtsignal
* rqt_robot_monitor/rqt_nav_view: modified to allow them to run properly as plugins after the move to rqt and renaming
* rqt_nav_view,rqt_robot_dashboard,rqt_robot_monitor: deleted stack.xml so they are part of the rqt stack (will catkinize later)
* rqt_pr2_dashboard and merged dependancies: Fixed instances of naming problems in code to allow rqt_pr2_dashboard to run again
* rqt_nav_view: initial import from external
  Squashing all the commits in the repository
  Map is now viewable.
  Now displays paths.
  Now accepts multiple paths.
  Fix README
  Better mirroring.
  Colors!
  Use objects not dictionaries for paths.
  Display polygons and tranform items properly.
  Nav view is now zoomable.
  Super important import.
  Use map_changed.
  Use ScrollHandDrag mode.
  Use wheel to scroll.
  Remove dumb zoom buttons.
  Cache the original map so reolution isnt lost.
  added window title
  Preparing nav_view for import into rqt
