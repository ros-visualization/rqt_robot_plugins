^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_robot_dashboard
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* rqt_robot_dashboard) comment added and PEP8 in MonitorDashWidget

0.2.15 (2013-04-25)
-------------------
* 0.2.15
* rqt_robot_dashboard) unit test for the pull req
  https://github.com/ros-visualization/rqt_robot_plugins/pull/21.
* Many pkgs) Corret and clean up dependencies.
* 0.2.14
* battery_dash_widget: Tooltip states name of battery.
  battery_dash_widget: float values are rounded.

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
* rqt_robot_dashboard: fixes rqt_pr2_dashboard `#7 <https://github.com/130s/rqt_robot_plugins/issues/7>`_
* rqt_robot_dashboard: fixes rqt_pr2_dashboard `#6 <https://github.com/130s/rqt_robot_plugins/issues/6>`_
* rqt_robot_dashboard: some names changed in imported packages

0.2.8 (2013-01-11)
------------------
* fix urls in package.xml files
* rqt_robot_dashboard: Nav view should be grey.
* rqt_robot_dashboard: Add accessor for widget state.
* Add orange background and led icon.
* rqt_robot_dashboard: Setup rosdoc and improve docs.

0.2.7 (2012-12-23 15:58)
------------------------

0.2.6 (2012-12-23 01:57)
------------------------

0.2.5 (2012-12-21 19:11)
------------------------
* pep8 style changes

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
* rqt_robot_dashboard: menu widget should be grey.

0.1.7 (2012-12-13 16:17)
------------------------

0.1.6 (2012-12-13 14:43:47)
---------------------------
* rqt_robot_monitor, rqt_robot_dashboard) (1) API (func names) changed for shutdown procedure, (2) funcs & variables names update (pub & private were wrongly declared before), (3) a few issues on the timeline on popup windows fixed (updated twice at a second, old values not available).

0.1.5 (2012-12-08)
------------------
* style and cleanup only
* rqt: removing incorrect entries in gitignore files
* rqt_robot_monitor/_dashboard) API name changed (conformed to PEP8).
* rqt_robot_monitor: fixed save_settings function to work upon close if the monitor is currently open
* rqt_robot_dashboard: fixed corner case
* rqt_pr2_dashboard: implemented save/restore of subwidget states
* rqt_dashboard: Battery now displays charging icons correctly
* rqt_robot_dashboard: interum hack to increase stability of pr2_dashboard
* rqt_robot_dashboard: Fixed possible recursive exception loop
* rqt_robot_dashboard: fixed missing block descriptors
* rqt_robot_monitor: fixes `#25 <https://github.com/130s/rqt_robot_plugins/issues/25>`_ (this bug is intermitant but is not reproduceable at this point)
* rqt_robot_dashboard: style change
* rqt_robot_monitor: console/monitor button fixes

0.1.4 (2012-11-20)
------------------

0.1.3 (2012-11-19)
------------------
* rqt_robot_dashboard: added icon missing icon and renamed all non-compliantly named icons
* rqt_robot_dashboard: mistake in gitignore file corrected
* rqt_robot_dashboard: icon update missing breaker, navigation and breaker 1 icons
* rqt_robot_dashboard: Style changes to allow users to access individual widgets without pulling in the dependancies for all
* rqt_robot_dashboard: old version of monitor shutdown call removed
* rqt_pr2/robot_dashboard: shutdown_widget hooks added to allow for widgets that need to cleanup dynamically allocated resources
* rqt_pr2/robot_dashboard: Added support for normal mode in dashboard consoles
* rqt_pr2/robot_dashboard: refactored icon generation code to live in utils.py instead of inside the IconToolButton, moved images arround to reflect that our default is now svg, allow image folder overlay ability for buttons
* rqt_pr2/robot_dashboard: removed duplicated code
* rqt_pr2/robot_dashboard: commenting/documentation overhaul, integration of new place holder icons, finalized migration to svg, style tweaks, translation added
* rqt_robot_dashboard: placeholder icons for future stock icons
* rqt_robot_dashboard: renamed icons to not be turtlebot branded
* rqt_robot_dashboard: fix to comply with modification of rqt_robot_monitor class name and constructor parameters
* rqt_pr2_dashboard/rqt_dashboard: refactor of icon code for ease of use and readability. Added doc strings and comments
* rqt_pr2_dashboard/rqt_robot_dashboard: refactor of icon code for ease of use with the build_icon function.
* rqt_robot_monitor: Update badges to fix color and centering
* rqt_robot_dashboard: removed extranious argument
* rqt_robot_dashboard: update battery Icon to fix missing portion
* rqt_robot_dashboard: icon pixel perfect alignment fix
* rqt_robot_dashboard: the robot_monitor widget now closes rqt_robot_monitor when it is not showing
* rqt_robot_dashboard/rqt_pr2_dashboard: new svg icon set
* rqt_nav_view,rqt_robot_dashboard,rqt_robot_monitor: Fixed the "old catkin version" and put these back to the previous system to be properly catkinized in the future
* rqt_pr2_dashboard: integrated new svg icons (Qt handles SVG differently than other formats), tweeks to functionality to bring functionality in line with original
* rqt_robot_dashboard: modified image load/icon functionality to allow an unlimited number of overlaid images to create an icon using QPainter instead of PIL
* rqt_nav_view,rqt_robot_dashboard,rqt_robot_monitor: deleted stack.xml so they are part of the rqt stack (will catkinize later)
* rqt_pr2_dashboard and merged dependancies: Fixed instances of naming problems in code to allow rqt_pr2_dashboard to run again
* rqt_robot_dashboard: initial import from external
  Initial stack setup.
  Basic plugin setup.
  Don't export a plugin.
  Added basic robot_monitor widget.
  Improved robot_monitor. Still need to remove items dynamically from tree.
  Added inspector with snapshot function.
  Split robot monitor in to seperate stack.
  Fix catkin python setup.
  Added basic API and a RobotMonitorWidget.
  Forgot to add robot_monitor_widget.
  Refactoring and add console widget.
  Fixed issue with widgets not closing.
  Dashboard is now a toolbar.
  Added a basic menu widget.
  Added basic sphinx docs.
  Started writing documentation.
  Move all widgets into one widgets.py file.
  More documentation.
  Basic test icons.
  Added a simple button widget.
  Use a proper toolbar.
  Added a state util push buttons now display state.
  Menus have state too.
  Consolidate set_state action into make_stately.
  sig_state accepts one int.
  Toolbars should get a name.
  Monitor displays state.
  Use context add_widget for robot monitor.
  Use lists not dictionaries.
  Basic progres bar based battery widget.
  Added on close.
  Setup before anything else happens.
  Destroyed not on_close.
  Added on click for convinience.
  Add a bunch of utils.
  Monitor now uses new icon system.
  Docs for IconToolButton.
  Console and battery now use new icon system.
  IconToolButton now generates a default icon set.
  Update state does not override button press.
  Proper shutdown.
  Need a 0% battery icon.
  Menu widget now uses new icon system. Needs icon.
  Remove stately and update docs.
  Preliminary mode icons.
  Remove old images.
  Added nav icons.
  Rename nav icons.
  This dummy didn't put the nav images in images.
  Added nav_view widget type.
  Docs docs docs!
  Update manifest.xml
  nav_view dependency was missing.
  New python binding system
  added zero battery support
  added missing dependancy
  cleanup for battery update
  added separator support to MenuDashWidget, enabled toggling on console/monitor buttons and console logging from launch of program
  added tooltips
  Added ability to update tooltip with # of console messages and changed the icon state to indicate the last 30 seconds of console messags
  fixed the issue with the icon not appearing initially
  added title support to Dashboard and allow children to set self.name in their setup functions
  Minor fix for name setting.
  fixed integer rounding error
  fixed issue with pyside portability
  Setting repository up for merge with rqt
  Conflicts:
  rqt_robot_dashboard/.gitignore
  rqt_robot_dashboard/README.md
