[![Toposens](https://toposens.com/wp-content/themes/toposens/assets/img/logo2.png)](https://toposens.com)

[![pipeline status](https://gitlab.com/toposens/ros-projects/ts-ros/badges/master/pipeline.svg)](https://gitlab.com/toposens/ros-projects/ts-ros/commits/master)


# Overview

ROS packages for working with Toposens 3D Ultrasound sensors: http://toposens.com/

Developed and tested for [ROS Melodic](http://wiki.ros.org/melodic) on [Ubuntu 18.04 (Bionic)](http://releases.ubuntu.com/18.04/)


# Setup

### For Ubuntu 18.04 (and maybe other versions)

 *  Follow [this](http://wiki.ros.org/melodic/Installation/Ubuntu) guide to install ROS on your system
 
 *  Install missing dependencies
    
    `sudo apt install python-catkin-tools ros-melodic-rviz-visual-tools ros-melodic-code-coverage`


# Building

 *  Follow Section 3 of [this](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) guide to create your catkin workspace

 *  Clone this repo into your catkin workspace
    
    `git@gitlab.com:toposens/ros-projects/ts-ros.git`

 *  Build your workspace from inside your catkin folder

    `catkin build toposens`


# Running

### Launching the Driver

 *  Make sure you are part of the dialout group:
 
    `sudo adduser $USER dialout`
 
 *  Trigger updated group permissions to take effect:

    `newgrp dialout`

 *  Launch the driver and start accruing data from a TS sensor
 
    `roslaunch toposens_driver toposens_driver.launch`
 
 *  To manipulate sensor parameters live in realtime, run in a new terminal

    `rosrun rqt_reconfigure rqt_reconfigure`
 
---
### Launching the Markers

 *  Launch the markers node:
 
    `roslaunch toposens_markers toposens_markers.launch`
 
 *  To manipulate marker parameters live in realtime, run in a new terminal:

    `rosrun rqt_reconfigure rqt_reconfigure`
 
---
### Launching the Pointcloud
 
 *  Start the driver:

    `roslaunch toposens_driver toposens_driver.launch`
 
 *  Launch the pointcloud in a new terminal window:

    `roslaunch toposens_pointcloud toposens_cloud.launch`

 *  To manipulate pointcloud parameters live in realtime, run in a new terminal:
 
    `rosrun rqt_reconfigure rqt_reconfigure`

---
### Turtlebot Integration

 *  SSH into your Turtlebot's onboard computer and start the special driver node:

    `roslaunch toposens_driver toposens_turtlebot.launch`
 
 *  From the remote computer, launch the special pointcloud file:

    `roslaunch toposens_pointcloud turtlebot_cloud.launch`

[![pipeline status](https://gitlab.com/toposens/ros-projects/ts-ros/badges/master/pipeline.svg)](https://gitlab.com/toposens/ros-projects/ts-ros/commits/master)


# Overview

ROS packages for working with Toposens 3D Ultrasound sensors: http://toposens.com/

Developed and tested for [ROS Melodic](http://wiki.ros.org/melodic) on [Ubuntu 18.04 (Bionic)](http://releases.ubuntu.com/18.04/)


# Setup

### For Ubuntu 18.04 (and maybe other versions)

 *  Follow [this](http://wiki.ros.org/melodic/Installation/Ubuntu) guide to install ROS on your system
 
 *  Install missing dependencies
    
    `sudo apt install python-catkin-tools ros-melodic-rviz-visual-tools ros-melodic-code-coverage`


# Building

 *  Follow Section 3 of [this](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) guide to create your catkin workspace

 *  Clone this repo into your catkin workspace
    
    `git@gitlab.com:toposens/ros-projects/ts-ros.git`

 *  Build your workspace from inside your catkin folder

    `catkin build toposens`


# Running

### Launching the Driver

 *  Make sure you are part of the dialout group:
 
    `sudo adduser $USER dialout`
 
 *  Trigger updated group permissions to take effect:

    `newgrp dialout`

 *  Launch the driver and start accruing data from a TS sensor
 
    `roslaunch toposens_driver toposens_driver.launch`
 
 *  To manipulate sensor parameters live in realtime, run in a new terminal

    `rosrun rqt_reconfigure rqt_reconfigure`
 
---
### Launching the Markers

 *  Launch the markers node:
 
    `roslaunch toposens_markers toposens_markers.launch`
 
 *  To manipulate marker parameters live in realtime, run in a new terminal:

    `rosrun rqt_reconfigure rqt_reconfigure`
 
---
### Launching the Pointcloud
 
 *  Start the driver:

    `roslaunch toposens_driver toposens_driver.launch`
 
 *  Launch the pointcloud in a new terminal window:

    `roslaunch toposens_pointcloud toposens_cloud.launch`

 *  To manipulate pointcloud parameters live in realtime, run in a new terminal:
 
    `rosrun rqt_reconfigure rqt_reconfigure`

---
### Turtlebot Integration
We assume, that the Turtlebot has been already setup once and that you know its IP address (`<turtlebot_ip>`, e.g. 192.168.0.179).

 *  Clone the turtlebot repos into your catkin workspace:
    
    `git clone https://github.com/ROBOTIS-GIT/turtlebot3.git`

    `git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git`

 *  Build your workspace from inside your catkin folder:

    `catkin build turtlebot3_msgs turtlebot3`
    
 *  Get to know your own IP address:

    In a terminal run `ifconfig`. In the displayed messages look for `wlp4s0` and its `inet` address (`<master_ip>`, e.g. 192.168.0.118)
    
 *  Add neccessary ROS parameters to your machine's bashrc (`nano ~/.bashrc`):
    
    ```
    export TURTLEBOT3_MODEL='burger'
    export ROS_MASTER_URI=http://<master_ip> :11311
    export ROS_HOSTNAME=<turtlebot_ip>
    ```
    
 *  SSH into your Turtlebot's onboard computer:
 
    `ssh pi@<turtlebot_ip>`

 *  Check and eventually adapt neccessary ROS parameters in the turtlebot's bashrc (`cat ~/.bashrc`, `nano ~/.bashrc`):
    
    ```
    export TURTLEBOT3_MODEL='burger'
    export ROS_MASTER_URI=http://<master_ip> :11311
    export ROS_HOSTNAME=<turtlebot_ip>
    ```

 *  On the turtlebot start the special driver node:

    `roslaunch toposens_driver toposens_turtlebot.launch`
 
 *  From the remote computer, launch the special pointcloud file:

    `roslaunch toposens_pointcloud turtlebot_cloud.launch`
