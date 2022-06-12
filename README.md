SOFAR_Assignment
=================

Developed by [Emanuele Rambaldi](https://github.com/LaRambla20), [Luca Mosetti](https://github.com/mose247) and [Francesco Ferrazzi](https://github.com/FraFerrazzi), June 2022.

The repository contains a possible solution to the final assignment of Software Architecture for Robotics course, hold at the MSc degree in Robotics Engineerg at the [University of Genova](https://unige.it/it/).

The goal of the project is to implement a simulation using ROS2 and the Nav2 stack to make a robot following a moving target at a fixed distance. The robots used in the simulation are two differential drive mobile robots. 

One robot is the "leader" which is driven manually by the `teleop_twist_keyboard` node. The other one is the "follower" which uses some features of [Nav2](https://navigation.ros.org/) to keep a fixed distance from the "leader".

The simulation is developed using [Gazebo](https://gazebosim.org/home) as simulation environment.

The result is reported in the brief video below:

Table of Contents
----------------------

- [SOFAR_Assignment](#sofar_assignment)
 * [Table of Contents](#table-of-contents)
 * [Installations Required](#installations-required)
 * [How to Run](#how-to-run)
 * [Documentation](#documentation)
 
 
Installations Required
----------------------

The project requires a [ROS2 Galactic](https://docs.ros.org/en/galactic/Installation.html) installation and the following packages if they are not already installed:

* for Ubuntu 20.04, install Nav2 by typing on terminal:
```bash
$ sudo apt install ros-galactic-navigation2 ros-galactic-nav2-bringup '~ros-galactic-turtlebot3-.*'
```
After installing Nav2, it is also required to build it. Go check the [linked page](https://navigation.ros.org/build_instructions/index.html) for this purpose.

* install xterm package by typing on terminal:
```bash
$ sudo apt-get install xterm
```
* install `teleop_twist_keyboard` by typing on terminal:
```bash
$ sudo apt-get install ros-galactic-teleop-twist-keyboard
```
* install Gazebo if it has not been intalled already by typing on terminal:
```bash
$ curl -sSL http://get.gazebosim.org | sh
```
After downloading all the required packages, if something is still missing, go check the [linked page](https://automaticaddison.com/how-to-create-an-object-following-robot-ros-2-navigation/). 

How to Run
-------------

First of all it is neccessary to modify the `.bashrc` file in the root folder of your filesystem, by adding the following lines:
* export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<absolute path to your ROS2 workspace>/src/object_following_robot/models/
* source /usr/share/gazebo/setup.sh (Source Gazebo)
* source /opt/ros/galactic/setup.bash (Source ROS2 Galactic)

To run the program is sufficient to clone the [GitHub](https://github.com/FraFerrazzi/SOFAR_Assignment) repository using the following command:
```bash
$ git clone https://github.com/FraFerrazzi/SOFAR_Assignment.git
```
Be careful to clone the repository into a ros-like workspace and build the project using the command:
```bash
$ colcon build
```
The command must be launched in the root folder of the workspace.

Before running the project it is necessary to change some absolute paths, which are:
* inside the `param` folder, in `nav2_robot1_params.yaml` at line 64.
* inside the `rviz` folder, in `nav2_config.rviz` at line 293.
* inside the `rviz` folder, in `nav2_config_multiple_robots.rviz` at line 293.
This absolute paths must be changed with your own ones depending on where you placed the project in your filesystem.

In the root folder of the Ros2 workspace, RUN THE PROGRAM by typing on terminal:
```bash
$ ros2 launch object_following_robot multi_robot_simulation_launch.py
```

Documentation
-------------


