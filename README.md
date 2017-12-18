The DWL's rviz plugin messages
==============================================

Table of Contents
==============================================
1. [Introduction](#introduction)
2. [Software Overview](#software-overview)
3. [Building](#building)


Introduction
==============================================
The Dynamic Whole Body Locomotion library (DWL) describes a set of core functions targeted to developed, design, and deploy locomotion algorithms, i.e. planning, control, etc. DWL is core library used in many projects of the Dynamic Legged Systems Lab of Istituto Italiano di Tecnologia (for more details about the project see http://www.iit.it/en/advr-labs/dynamic-legged-systems.html). For more details about DWL software infrastructure please visit https://github.com/robot-locomotion/dwl.

The DWL rviz plugin contains a set of tools and plugins for visualization of: the whole-body state of the robot (dwl_rviz_plugin::WholeBodyStateDisplay), the whole-body trajectory (dwl_rviz_plugin::WholeBodyTrajectoryDisplay), the terrain map (dwl_rviz_plugin::TerrainMapDisplay), the footstep regions (dwl_rviz_plugin::FootstepRegionDisplay), etc. These plugins subscribe to the standarized dwl messages (for more information see https://github.com/robot-locomotion/dwl-msgs).

You can also have tools that allows us to create our custom plugin such as: points (dwl_rviz_plugin::PointVisual), lines (dwl_rviz_plugin::LineVisual), polygons (dwl_rviz_plugin::PolygonVisual), arrows (dwl_rviz_plugin::ArrowVisual), etc.

[![ScreenShot](https://imgur.com/Ox7pa0e.gif)](https://www.youtube.com/watch?v=ENHvCGrnr2g)

The source code is released under a [BSD 3-Clause license](LICENSE).


Software Overview
==============================================
The algorithms are built primarily in C/C++. The library uses a number of the local dependencies, which some of them are optionals.

The dwl-rviz-pluin is a ROS packages with the following required dependencies:
* [Boost](http://www.boost.org) (version 1.5.4 or higher)
* [Rviz](http://www.cmake.org)
* [dwl](https://github.com/robot-locomotion/dwl)
* [dwl-msgs](https://github.com/robot-locomotion/dwl-msgs)

The following dependencies are optional:
* [Doxygen](http://www.doxygen.org)


Building
===============================================
Before building the dwl_msgs you need to install the dependencies of DWL. Additionally you have to build dwl with catkin.

The dwl_rviz_plugin is a catkin project which can be built as:

	cd your_ros_ws/
	catkin_make
