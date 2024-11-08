^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package leo_gz_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2024-11-08)
------------------
* Fix mypy errors (`#11 <https://github.com/LeoRover/leo_simulator-ros2/issues/11>`_) (`#12 <https://github.com/LeoRover/leo_simulator-ros2/issues/12>`_)
* Add launch_ros to dependencies
* Contributors: Błażej Sowa

2.0.0 (2024-04-27)
------------------
* Get rid of all the ignition references
* Contributors: Błażej Sowa

1.1.0 (2024-04-27)
------------------
* Set new GZ variables to work on Gazebo Harmonic
* Contributors: Błażej Sowa

1.0.0 (2023-11-08)
------------------
* Add ament_black test to leo_gz_bringup (`#4 <https://github.com/LeoRover/leo_simulator-ros2/issues/4>`_)
* Revert default world to empty world
* Fix wrong import and code formatting
* Add robot_ns argument to leo_gz.launch
* Add missing dependencies and better descriptions
* Fix image bridge working only on one robot
* Add namespaces to robot launch
* CMakeLists cleanup
* Remove flake8 test
* Add default gazebo version as fortress
* Update package.xml files and copyright notices
* Update copyright notices
* Format XML files
* Add copyright notices
* CI fix imports and whitespaces
* Fix CI failure for launch and cpp files
* Add CI (`#1 <https://github.com/LeoRover/leo_simulator-ros2/issues/1>`_)
* Change project versions
* Add package descriptions
* Code cleanup
* Add bridge for multiple robots
* Add separate launch file to spawn the robot
* Add worlds package
* Rename launchfile
* Parametrize sim world and add more camera features
* first semi-stable version
* Contributors: Błażej Sowa, Jan Hernas
