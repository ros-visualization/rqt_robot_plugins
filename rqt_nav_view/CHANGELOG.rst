^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_nav_view
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.2.15 (2013-04-25)
-------------------
* Corret and clean up dependencies.

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
* maintainer added

0.2.9 (2013-03-07)
------------------
* Ensured that nav_view_dash_widget removes the widget when clicked a second time

0.2.8 (2013-01-11)
------------------
* Fix
  * Map scrolling fixed.
  * Drag and drop works everywhere now.
  * Fix map colors.
* Enhancement
  * NavFn should be the default.
  * Goals can now be set.
  * Initial pose can now be set.
  * Don't update without map.
  * Don't regenerate unless map changes.
  * Only use one tranform listener.
  * Topics can be drag and dropped.
* Documentation
  * fix urls in package.xml file

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
* Fix
  * fix deps
  * fix export tag in rqt_gui_py
  * Removed extra plugin install rule.
* Enhancement
  * add missing target_link_libraries
  * disable graying out unavailable python plugins
* Refactoring
  * catkinizing
  * catkinize shell scripts and removed load_manifest calls
  * run from their scripts in install space

0.1.7 (2012-12-13 16:17)
------------------------

0.1.6 (2012-12-13 14:43:47)
---------------------------

0.1.5 (2012-12-08)
------------------
* Fix
  * fixed failure to unsubscribe to topics
* Enhancement
  * moved from scaling zoom to viewscaling zoom
* Refactoring
  * port to use native QImages instead of PIL images due to PIL being PyQt only
  * Added Exception and explaination of the limitation of one plugin per rqt instance

0.1.4 (2012-11-20)
------------------

0.1.3 (2012-11-19)
------------------
* initial import from external
