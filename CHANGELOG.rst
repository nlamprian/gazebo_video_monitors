^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_video_monitor_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2020-06-29)
------------------
* Refactor gazebo video monitor plugin
  * Introduce gazebo video recorder to host recording functionality and enable reusability
  * Introduce gazebo monitor base plugin to host common members and structure plugin initialization
  * Add option to disable the window in the gazebo video monitor plugin
  * Update documentation
* Drop specific OpenCV version
* Parameterize log prefixes

0.1.1 (2020-04-23)
------------------
* Set path of temporary recording
* Add fix for initial camera attachment

0.1.0 (2020-04-21)
------------------
* Add multicamera sensor and video monitor plugin
