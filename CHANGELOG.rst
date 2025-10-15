^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package trimble_gnss_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.5 (2025-10-15)
------------------
* ROS2 version
* Contributors: Michael Hutchinson, MikHut

0.0.4 (2023-05-19)
------------------
* Merge pull request `#4 <https://github.com/SAGARobotics/trimble_gnss_driver/issues/4>`_ from polymathrobotics/master
  Improve read reliability over poor network connectivity
* Merge branch 'devel' into 'master'
  Improve read reliability over poor network connectivity
  See merge request polymathrobotics/third_party/trimble_gnss_driver!1
* Improve read reliability over poor network connectivity
* Remove unused include
* Contributors: Michael Hutchinson, MikHut, Will Baker

0.0.3 (2022-07-07)
------------------
* Merge branch 'master' of https://github.com/SAGARobotics/trimble_gnss_driver
* Position covariance reorder to be ENU
* Contributors: MikHut

0.0.2 (2022-07-07)
------------------
* get position type from receiver msg 38
* Contributors: MikHut

0.0.1 (2022-07-04)
------------------
* remove unused deps
* remove unused deps
* Merge pull request `#3 <https://github.com/MikHut/trimble_gnss_driver/issues/3>`_ from SAGARobotics/Add-launch-file-to-cmakelists
  Update CMakeLists.txt
* Update CMakeLists.txt
* Create internal-release.yml (`#2 <https://github.com/MikHut/trimble_gnss_driver/issues/2>`_)
  * Create internal-release.yml
* Update ci.yml
* Update ci.yml
* Create ci.yml
* Merge pull request `#1 <https://github.com/MikHut/trimble_gnss_driver/issues/1>`_ from MikHut/base_name
  Base name
* python version independant
* checksum fixed
* type 35 msg working, base info, does not work for rtcm though
* initial test for getting the base station name currently used
* GPL license added
* Update package.xml
* tf exec depend
* print heading offset applied to yaw output
* tidy prints, just print warning when msgs are being skipped from being published due to no matching error or invalid data
* option to specifically name the output frame_id and ability to offset the heading using the robot urdf if desired
* dont publish if checksum is false
* move gps qualities dict out of main code
* cleanup
* update comment in launch file
* send attitude as imu msg type instead
* tidying, dont publish zeros, publish yaw on an Imu msg if from dual antenna with no fusion
* added publishers, made nice parse, tidied, sorted pitch coord system for ros
* updates to test on robot
* small fix
* updates
* work in progress, initial setup to test on robot
* Initial commit
* Contributors: Bård-Kristian, Michael Hutchinson, MikHut
