# ENPM808x ROS Part 1 - ROS Publisher/Subsciber example

[![Packagist](https://img.shields.io/packagist/l/doctrine/orm.svg)](LICENSE.md)

## Overview of the project

ROS Publisher/Subsciber example following the official ROS Tutorials as given here (http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29).

## Dependencies

The following dependencies are required to run this package:

1. ROS kinetic
2. catkin (http://wiki.ros.org/catkin#Installing_catkin)
3. Ubuntu 16.04 For installing ROS (http://wiki.ros.org/kinetic/Installation)

## Standard install via command-line
```
cd ~/catkin_ws/src
mkdir beginner_tutorials
cd beginner_tutorials
git clone --recursive https://github.com/arp95/beginner_tutorials
cd ../..
catkin_make
```

Open three terminals and run the following commands in them:

1. Terminal 1:
```
roscore
```

2. Terminal 2:
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials talker
```

3. Terminal 3:
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener
```
