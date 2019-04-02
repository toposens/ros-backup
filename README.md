[![Toposens](https://toposens.com/wp-content/themes/toposens/assets/img/logo2.png)](https://toposens.com)

[![pipeline status](https://gitlab.com/toposens/ros-projects/ts-ros/badges/master/pipeline.svg)](https://gitlab.com/toposens/ros-projects/ts-ros/commits/master)
[![coverage report](https://gitlab.com/toposens/ros-projects/ts-ros/badges/master/coverage.svg)](https://gitlab.com/toposens/ros-projects/ts-ros/commits/master)

---
=======
# Overview

## Dependencies

### For Ubuntu 18.04 (and maybe other versions)

 Install ROS using this guide [HERE](http://wiki.ros.org/melodic/Installation/Ubuntu).

 After that make sure the missing dependencies are met by running:
 
 `sudo apt-get install python-catkin-tools ros-melodic-rviz-visual-tools ros-melodic-code-coverage`

## Building

 Follow the instructions [HERE](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) under section 3.

 After that clone the repo into your catkin-workspace.

 Build by running inside your workspace-folder:
 
 `catkin build toposens`

## Launching

### Launching the marker

 To launch the marker run:

 `roslaunch toposens_markers toposens_markers.launch`

### Launching the pointcloud

 To launch the pointcloud:

 1st Terminal:

 `roslaunch toposens_pointcloud toposens_cloud.launch`

 2nd Terminal:

 `roslaunch toposens_driver toposens_driver.launch`

 To manipulate parameters live without restarting:

 3rd Terminal:

 `rosrun rqt_reconfigure rqt_reconfigure`

---
