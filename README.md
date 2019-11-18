# ROS Turtlebot Walker

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Project overview
Roomba vacuum cleaner robot like implementation on turtlebot by avoiding obstacles in space.

## Dependencies

This ROS node is made to be used on systems which have:

    ROS Kinetic
    Ubuntu 16.04
    Turtlebot packages


To install the turtlebot packages, run the following after installing ROS Kinetic on your ubuntu 16.04.

```
sudo apt-get install ros-kinetic-turtlebot-*
```


##  Build
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/kartikmadhira1/turtlebot_walker
cd ..
catkin_make
```
## Run demo
In order to launch the turtlebot simulation, type the command below:

```
roslaunch turtlebot_walker turtle_walker.launch
```
If you'd like to run the nodes separately, then run commands below in seperate terminals
```
roscore
```
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```
```
rosrun turtlebot_walker turtlebot_walker
```
## Recording bag files with the launch file

To launch the recording all topics excepth the camera data, use the command below:
```
roslaunch turtlebot_walker turtle_walker.launch record:=true record_time:=32
```
The bag file is stored in 
```
..turtlebot_walker/results/turtle_walker.bag
```
## Playing back the bag file

To play the rosbag file recordings:
```
rosbag play results/turtlebot_walker.bag
```
You can view the messages being published on the topic /cmd_vel_mux/input/navi.
```
rostopic echo /cmd_vel_mux/input/navi
```