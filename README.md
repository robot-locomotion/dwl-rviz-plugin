The dwl-rviz-plugin
==============================================

## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Introduction

The *Dynamic Whole-body Locomotion* library (DWL) describes a set of core functions targeted to developed, design, and deploy locomotion algorithms, i.e. planning, control, etc. DWL is core library used in many projects of the Dynamic Legged Systems Lab of Istituto Italiano di Tecnologia (for more details about the project see http://www.iit.it/en/advr-labs/dynamic-legged-systems.html). For more details about DWL software infrastructure please visit https://github.com/robot-locomotion/dwl.

The dwl_rviz_plugin contains a set of tools and plugins for visualization of: the whole-body state of the robot (dwl_rviz_plugin::WholeBodyStateDisplay), the whole-body trajectory (dwl_rviz_plugin::WholeBodyTrajectoryDisplay), the terrain map (dwl_rviz_plugin::TerrainMapDisplay), the footstep regions (dwl_rviz_plugin::FootstepRegionDisplay), etc. These plugins subscribe to the standarized dwl messages (for more information see https://github.com/robot-locomotion/dwl-msgs).

You can also have tools that allows us to create our custom plugin such as: points (dwl_rviz_plugin::PointVisual), lines (dwl_rviz_plugin::LineVisual), polygons (dwl_rviz_plugin::PolygonVisual), arrows (dwl_rviz_plugin::ArrowVisual), etc.

| [![](https://i.imgur.com/oiBL7Be.gif)](https://www.youtube.com/watch?v=MOhJRMqBWQk&feature=youtu.be) | [![](https://i.imgur.com/4kKhryj.gif)](https://www.youtube.com/watch?v=KI9x1GZWRwE)
|:-------------------------:|:-------------------------:|
| [![](https://i.imgur.com/TUrgkzO.gif)](https://www.youtube.com/watch?v=MOhJRMqBWQk&feature=youtu.be) | [![](https://i.imgur.com/RKe3sNo.gif)](https://www.youtube.com/watch?v=KI9x1GZWRwE)
|||

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Carlos Mastalli, carlos.mastalli@laas.fr<br />
With support from the Dynamic Legged Systems lab at Istituto Italiano di Tecnologia<br />**



## <img align="center" height="20" src="https://i.imgur.com/fjS3xIe.png"/> Dependencies

The algorithms are built primarily in C++. The library uses a number of the local dependencies, which some of them are optionals.

The dwl-rviz-plugin is a ROS packages with the following required dependencies:
* [Boost](http://www.boost.org) (version 1.5.4 or higher)
* [Rviz](http://www.cmake.org)
* [dwl](https://github.com/robot-locomotion/dwl)
* [dwl-msgs](https://github.com/robot-locomotion/dwl-msgs)

The following dependencies are optional:
* [Doxygen](http://www.doxygen.org)



## <img align="center" height="20" src="https://i.imgur.com/x1morBF.png"/> Building

Before building the dwl_rviz_plugin, you need to install the dependencies of DWL. Additionally you have to build dwl and dwl_msgs with catkin.

The dwl_rviz_plugin is a catkin project which can be built as:

	cd your_ros_ws/
	catkin_make

