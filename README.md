SOFAR_Assignment
=================

Developers: [Emanuele Rambaldi](https://github.com/LaRambla20), [Luca Mosetti](https://github.com/mose247) and [Francesco Ferrazzi](https://github.com/FraFerrazzi).

This is the assignment of the course Software Architecture for Robotics, provided by [Università Degli Studi di Genova](https://unige.it/it/), Robotics Engineering degree.

The goal of this assignment is to build a simulation with Ros2 in which a robot follows a moving target at a fixed distance.
The robots used in the simulation are two four wheeled, differential drive, mobile robots. 

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

The project requires a [Ros2 Galactic](https://docs.ros.org/en/galactic/Installation.html) installation and the following packages if they are not already installed:

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

To run the program is sufficient to clone the [GitHub](https://github.com/FraFerrazzi/SOFAR_Assignment) repository using the following command:
```bash
$ git clone https://github.com/FraFerrazzi/SOFAR_Assignment.git
```
Be careful to clone the repository into a ros like workspace and build the project using the command:
```bash
$ colcon build
```
The command must be launched in the root folder of the workspace.

Before running the project it is necessary to change some absolute paths, which are:
* inside the `param` folder, in `nav2_robot1_params.yaml` at line 64.
* inside the `rviz` folder, in `nav2_config.rviz` at line 293.
* inside the `rviz` folder, in `nav2_config_multiple_robots.rviz` at line 293.
This absolute paths must be changed with your own ones depending on where you place the project in your architecture.

In the root folder of the Ros2 workspace, RUN THE PROGRAM by typing on terminal:
```bash
$ ros2 launch object_following_robot multi_robot_simulation_launch.py
```

Documentation
-------------

