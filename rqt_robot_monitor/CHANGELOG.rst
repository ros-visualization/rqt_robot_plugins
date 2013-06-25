^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_robot_monitor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.2.15 (2013-04-25)
-------------------
<<<<<<< HEAD
* Corret and clean up dependencies.
* Correct missing/unnecessary dependency.
=======
* 0.2.15
* Many pkgs) Corret and clean up dependencies.
* rqt_robot_monitor) Correct missing/unnecessary dependency.
* 0.2.14
>>>>>>> d118fd8a32da5275d6c16116d3ba617264b3ca70

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
* Enhancement
  * minor refactoring, cleaning.
* Documentation
  * modified pkg description. Authors added. Modify email address

0.2.9 (2013-03-07)
------------------
* Fix
  * now run with pyside (it used to be not working with it w/o having been noticed).
  * Call .ui file in .ui is now successfully working
* Refactoring
  * Now TimelinePane uses .ui file
  * Complete PEP8 adjustment
* Documentation
  * Add docroot to public methods.
  * Minor (code repo url specified more)

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
* catkinizing

0.1.7 (2012-12-13 16:17)
------------------------

0.1.6 (2012-12-13 14:43:47)
---------------------------
* Fix
  * a few issues on the timeline on popup windows fixed (updated twice at a second, old values not available)
* Refactoring
  * API (func names) changed for shutdown procedure
  * funcs & variables names update (pub & private were wrongly declared before)
  * Undesirable callback hook is corrected.

0.1.5 (2012-12-08)
------------------
* Fix
  * 2 major bugs fixed. (1) causes rqt_gui to segfaults when more Error/Warn signals are received (5+ when I observed), (2) Thread alive even after shutting down the plugin which sometimes end up error close.
  * timeline doesn't adjuts its size when paused
  * timeline often shows blank cell on the right end
  * fix for timeline resizing issue
  * Icons switched to the ones that should be more common.
  * Icon for the status STALE not appearing on Error pane.
* Enhance
  * time elapsed after the last topic/msg received is now shown.
  * Warn & Error trees now show only the most detail (ie. abstracted warn/error content no longer shown) to provide quicker access to the specific detail.
  * Added guide text on .ui file (about timeline's direction)
  * API name changed (conformed to PEP8).
  * introspection by timeline. Also merged with the latest push.

0.1.4 (2012-11-20)
------------------

0.1.3 (2012-11-19)
------------------
* Initial commit
