# Software Development for Robotics (ENPM808x) ROS Exercise

[![Packagist](https://img.shields.io/packagist/l/doctrine/orm.svg)](LICENSE.md)


## Overview of the project

1. ROS Publisher/Subsciber example following the official ROS Tutorials as given here (http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29).

2. ROS Services and Logging example following the official ROS Tutorials as given here (http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams).

3. ROS tf and bag files example following the official ROS Tutorials as given here (http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf).


## Dependencies

The following dependencies are required to run this package:

1. ROS kinetic
2. catkin (http://wiki.ros.org/catkin#Installing_catkin)
3. Ubuntu 16.04 For installing ROS (http://wiki.ros.org/kinetic/Installation)


## Standard install via command-line
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/arp95/beginner_tutorials
cd ..
catkin_make
```

Open the following terminals and run the following commands in them:

1. Terminal 1:
```
roscore
```

2. Terminal 2:
Passing param=error and not recording bag file:
```
cd catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials beginner_tutorials.launch param:=error record:=false
```

Passing param=warn and record bag file:
```
cd catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials beginner_tutorials.launch param:=warn record:=true
```

Passing param=fatal and record bag file:
```
cd catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials beginner_tutorials.launch param:=fatal record:=true
```

Passing param=warn and record bag file:
```
cd catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials beginner_tutorials.launch param:=warn record:=true

3. Terminal 3 (inspect tf frames, provided talker node is running):
```
cd catkin_ws
source devel/setup.bash
rosrun rqt_tf_tree rqt_tf_tree
```

3. Terminal 4 (store tf frames output, provided talker node is running):
```
cd catkin_ws
source devel/setup.bash
rosrun tf view_frames
```

Output:
```
Listening to /tf for 5.000000 seconds
Done Listening
dot - graphviz version 2.38.0 (20140413.2041)

Detected dot version 2.38
frames.pdf generated
```


## Print information in a bag file
1. Terminal 1:
```
cd catkin_ws
source devel/setup.bash
cd src/beginner_tutorials/results
rosbag info beginner_tutorials.bag
```

Output:
```
path:        src/beginner_tutorials/results/beginner_tutorials.bag
version:     2.0
duration:    38.7s
start:       Nov 06 2019 21:24:54.70 (1573093494.70)
end:         Nov 06 2019 21:25:33.43 (1573093533.43)
size:        585.3 KB
messages:    3021
compression: none [1/1 chunks]
types:       rosgraph_msgs/Log  [acffd30cd6b6de30f120938c17c593fb]
             std_msgs/String    [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage [94810edda583a504dfda3829e70d7eec]
topics:      /chatter       385 msgs    : std_msgs/String   
             /rosout       1127 msgs    : rosgraph_msgs/Log  (3 connections)
             /rosout_agg   1124 msgs    : rosgraph_msgs/Log 
             /tf            385 msgs    : tf2_msgs/TFMessage
```


## Test Talker Node
1. Terminal 1:
(to launch test.launch)
```
cd catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials test.launch
```

(to test the service of adding two numbers in talker node)
```
cd catkin_ws
source devel/setup.bash
rostest beginner_tutorials test.launch
```

Output:
```
[ROSUNIT] Outputting test results to /home/arpitdec5/.ros/test_results/beginner_tutorials/rostest-test_test.xml
[ERROR] [1573487396.226003612]: ERROR Logger Level.
[FATAL] [1573487396.426060810]: FATAL Logger Level.
[Testcase: testtalkerNodeTest] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-talkerNodeTest/testServiceName][passed]
[beginner_tutorials.rosunit-talkerNodeTest/testServiceOutput][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/arpitdec5/.ros/log/rostest-arpitdec5-23879.log
```


## Listener Node demostration
1. Terminal 1 (run master node):
```
roscore
```

2. Terminal 2 (play rosbag):
```
cd catkin_ws
source devel/setup.bash
rosbag play src/beginner_tutorials/results/beginner_tutorials.bag
```

3. Terminal 3 (run listener node):
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener
```
