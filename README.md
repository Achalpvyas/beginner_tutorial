# ROS Publisher Subscriber Beginner Tutorials
[![License](https://img.shields.io/badge/License-BSD%203--Clause-orange.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview

This beginner_tutorials creates publisher and subscriber nodes and transfer messages between them.

## Dependencies


 Ubuntu 16.04 Xenial 
 ROS kinetic 
 Catkin

## Install ROS

```
~$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
~$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
~$ sudo apt-get update
~$ sudo apt-get install ros-kinetic-desktop-full
~$ sudo rosdep init
~$ rosdep update
~$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
~$ source ~/.bashrc
~$ source /opt/ros/kinetic/setup.bash
~$ sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

## Building Workspace and Packages

```
~$ mkdir -p ~/catkin_ws
~$ cd catkin_ws
~$ mkdir src
~$ catkin_make
~$ source devel/setup.bash
~$ cd src
~$ git clone https://github.com/Achalpvyas/beginner_tutorials.git
~$ cd ..
~$ source devel/setup.bash 
```

## Run Publisher and Subscriber nodes
Publish, Subscribe and Examine
[link](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
[link](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)

```
~$ roscore

~$ cd catkin_ws
~$ rosrun beginner_tutorials talker <frequency>

~$ rosrun beginner_tutorials listener

```

## Launch file

```

~$ cd catkin_ws
~$ source devel/setup.bash
~$ roslaunch --screen beginner_tutorials Week10_HW.launch

```

## ROS Service

```
~$ rosservice call /ChangeString "what is your name"
```

## ROS Logger

```
rqt_console
rqt_logger_level


