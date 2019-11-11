# ROS Beginner Tutorials
[![License: BSD](https://opensource.org/licenses/BSD-3-Clause)

## Overview
Testing tf, bag files.

## Dependencies

 ROS kinetic 
 Catkin 
 Ubuntu 16.04 Xenial 

## Install ROS

```
Open a terminal
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

### Workspace and Packages
Use the following commands on the terminal to build the workspace and the package with the relevant dependencies.

```
~$ mkdir -p ~/catkin_ws
~$ cd catkin_ws
~$ mkdir src
~$ catkin_make
~$ source devel/setup.bash
~$ cd src
~$ git clone https://github.com/Achalpvyas/beginner_tutorials
~$ cd ..
~$ catkin_make
~$ source devel/setup.bash 
```
## Using Launch

```
$ cd catkin_ws
$ source devel/setup.bash
$ roslaunch beginner_tutorials homework11.launch

```

## ROS Service
To change the string output replace "Changed String" to something.
```
Open a terminal
$ cd catkin_ws
$ source devel/setup.bash
$ roslaunch beginner_tutorials homework11.launch

Open a Terminal
$ cd catkin_ws
$ source devel/setup.bash
$ rosservice call /changeString "stream change"
```

## tf Frame verification

### Using tf_echo
```
Open a terminal

$ cd catkin_ws
$ source devel/setup.bash
$ rosrun tf tf_echo world talk
```

### Using view_frames
```
Open a terminal
$ cd catkin_ws
$ rosrun tf view_frames
```

## ROS Unit tests
Run the unit testing in the new terminal.
Open a terminal
### Testing using Launch
```
$ cd catkin_ws
$ source devel/setup.bash
$ rostest beginner_tutorials test.launch

```

## Bag files (Play and Record)

### Record
After launching the nodes

Open terminal
```
$ rosscore
```

Open terminal
```
$ cd catkin_ws
$ source devel/setup.bash
$ rosrun beginner_tutorials talker
```
Open terminal
```
$ cd catkin_ws
$ source devel/setup.bash
$ cd src/beginner_tutorials/Results
$ rosbag record -a

ctrl + c after 15 seconds to terminate
```

### Play

First we will run the listener node.

Open terminal 
```
$ cd catkin_ws
$ source devel/setup.bash
$ rosrun beginner_tutorials listener
```
Now will play the recorded talker bag file
Open terminal 
```
$ cd catkin_ws
$ source devel/setup.bash
$ cd src/beginner_tutorials/Results
$ rosbag play message_record.bag
```

## Google Styling
```
$ roscd beginner_tutorials
$ cd Results
```
