### Table of Contents

- [Assignment description](#assignment-description)
  * [Installations Required](#installations-required)
  * [How to Run](#how-to-run)
  * [Documentation](#documentation)
  * [Sources](#sources)

Assignment description
=================

Developed by [Emanuele Rambaldi](https://github.com/LaRambla20), [Luca Mosetti](https://github.com/mose247) and [Francesco Ferrazzi](https://github.com/FraFerrazzi), June 2022.

The repository contains a possible solution to the final assignment of Software Architecture for Robotics course, hold at the MSc degree in Robotics Engineerg at the [University of Genoa](https://unige.it/it/).

The goal of the project is to implement a simulation using ROS2 and the Nav2 navigation stack to make a robot following a moving target at a fixed distance. In particular, it has been decided to use two differential drive robots. The first behaves as the "follower", while the second as the "leader".

The simulation is developed using [Gazebo](https://gazebosim.org/home) as simulation environment.

The result is reported in the brief video below:

![ezgif-5-ee980ba315](https://user-images.githubusercontent.com/91455159/173257879-858cf26b-eed0-4dc6-ab5d-bf962255917e.gif)

 
Installations Required
----------------------

The project requires a [ROS2 Galactic](https://docs.ros.org/en/galactic/Installation.html) installation and some additional packages. If they are not already installed you can download them via terminal with the following commands.

*  install _Nav2_:
```bash
$ sudo apt install ros-galactic-navigation2 ros-galactic-nav2-bringup '~ros-galactic-turtlebot3-.*'
```
Please note that, after installing Nav2, it is also required to build it in a dedicated workspace. Go check the [official guide](https://navigation.ros.org/build_instructions/index.html) to this purpose.

* install _xterm_:
```bash
$ sudo apt-get install xterm
```
* install _teleop_twist_keyboard_:
```bash
$ sudo apt-get install ros-galactic-teleop-twist-keyboard
```
* install _Gazebo_:
```bash
$ curl -sSL http://get.gazebosim.org | sh
```
After having downloaded all the required packages, if something is still missing, go check the [linked page](https://automaticaddison.com/how-to-create-an-object-following-robot-ros-2-navigation/). 

How to Run
-------------

First of all, it is neccessary to modify the `.bashrc` file in the root folder of your filesystem, by adding the following lines:
* export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<absolute path to your ROS2 workspace>/src/object_following_robot/models/
* source /usr/share/gazebo/setup.sh (Source Gazebo)
* source /opt/ros/galactic/setup.bash (Source ROS2 Galactic)

To run the program is sufficient to clone the [GitHub](https://github.com/FraFerrazzi/SOFAR_Assignment) repository using the following command:
```bash
$ git clone https://github.com/FraFerrazzi/SOFAR_Assignment.git
```
Be careful to clone the repository into a ROS-like workspace and build the project using the command:
```bash
$ colcon build
```
The command must be launched in the root folder of the workspace.

Before running the project make sure to modify the absolute paths specified in the following files:
* inside `./param/nav2_robot1_params.yaml`, at line 64.
* inside `./rviz/nav2_config.rviz`, at line 293.
* inside `./rviz/nav2_config_multiple_robots.rviz`, at line 293.

These absolute paths must be changed with your own ones depending on where you placed the project in your filesystem.

In the root folder of the Ros2 workspace, RUN THE PROGRAM by typing on terminal:
```bash
$ ros2 launch object_following_robot multi_robot_simulation_launch.py
```

Documentation
-------------

In synthesis, the general software architecture is the following:
<img width="1532" alt="URL_diagram" src="https://user-images.githubusercontent.com/91314392/173240621-f51c2a4e-89b0-47fa-9798-9053395f240c.png">

The leading robot is user-guided via the `teleop_twist_keyboard` node, which publishes the desired Twist on the `/robot2/cmd_vel` topic. 

Instead, the following robot navigates autonomusly in the environment using the Nav2 navigation stack. Specifically, the navigation behavior is defined by a [behavior tree](https://navigation.ros.org/behavior_trees/index.html) aimed at following a dynamic point keeping a fixed distance.

As goal, the navigation stack takes the position and orientation of the leader with respect to the map frame, which is published on `/robot1/goal_pose`. To retrive such information, the node `robot2_pose_publisher` subcribes to the tobic `/robot2/tf` to compute the transformation between the map frame and the footprint of the second robot. 

Sources
--------

In order to develop the solution to the assignment the two following pre-built packages were used:
* `two_wheeled_robot` ([package link](https://drive.google.com/drive/folders/1JP12kp4JZ6SM0E8zhxI99ERBMf3qL6QW) - author: Addison Sears-Collins): package aimed at exploring the potentialities of the Nav2 navigation stack, by using a simple differential robot spawned in various worlds
* `nav2_bringup` ([package link](https://github.com/ros-planning/navigation2/tree/main/nav2_bringup) - author: Nav2 developers): package aimed at offering a bringup system for the Nav2 navigation stack applications

Specifically, starting from these two packages, the steps to obtain the final result were the following:
* create a new ROS2 package
* export the house_world simulation implemented in the `two_wheeled_robot` package to the newly-created package. In particular, the following elements were taken:
  * `house_world.pgm` and `house_world.yaml` from the folder `maps/house_world`
  * `nav2_params.yaml` from the folder `params/house_world`
  * `nav2_config.rviz` from the folder `rviz/house_world`
  * `two_wheeled_robot.urdf` from the folder `urdf`
  * `house.world` from the folder `worlds`
  * all the models of the objects spawned in the world from the folder `models` (differential robot model included)
* export the multi_turtlebot3 simulation implemented in the `nav2_bringup` package to the newly-created package. In particular, all the launch files contained in the folder `launch` were taken
* include in the `params` folder of the newly-created package the dynamic object following behaviour tree (which can be found at the following web page: [dynamic object following behaviour tree](https://navigation.ros.org/tutorials/docs/navigation2_dynamic_point_following.html))
* duplicate the `nav2_params.yaml` so as to have two different parameters files (`nav2_robot1_params.yaml` and `nav2_robot2_params.yaml`), one for each robot
* modify the two parameter files as follows:
  * `nav2_robot1_params.yaml` (contained in the `params` folder of the newly-created package):  
    * substitute 'topic: /scan' with 'topic: /robot1/scan'
    * comment out 'initial_pose: ...' and add 'initial_pose.x: 0.0', 'initial_pose.y: 0.0', 'initial_pose.z: 0.25', 'initial_pose.yaw: 0.0'
    * set 'default_nav_to_pose_bt_xml' to the absolute path to the dynamic object following behaviour tree
    * set 'enable_groot_monitoring' to False (in order to avoid the failure of the dynamic object following behaviour tree)
  * `nav2_robot2_params.yaml` (contained in the `params` folder of the newly-created package): 
    * substitute 'topic: /scan' with 'topic: /robot2/scan'
    * remove 'initial_pose: ...' and add 'initial_pose.x: 0.0', 'initial_pose.y: 1.0', 'initial_pose.z: 0.25', 'initial_pose.yaw: 0.0'
    * set 'enable_groot_monitoring' to False (in order to avoid the failure of the default behaviour tree)
* modify the rviz configuration file so as to allow for the handling of multiple robots:
  * `nav2_config_multiple_robots.rviz` (contained in the `rviz` folder of the newly-created package):
    * put '<robot_namespace>/' in front of the 'Value' parameter of each 'Topic' (e.g. Value: local_plan -> Value: <robot_namespace>/local_plan)
    * set the 'Description File' parameter to the absolute path to the urdf file of the robot
* modify the world description file so as to avoid spawning twice the robots in the environment:
  * `house.world` (contained in the `worlds` folder of the newly-created package):
    * remove the last lines of the files: lines that include the sdf model of the robot in the environment (this task is already carried out by the launch file) 
* modify the sdf model of the robot so as to publish the coordinate transform from odom to base_footprint by means of the default amcl localization algorithm:
  * `model.sdf` (contained in the `models/two_wheeled_robot_description` folder of the newly-created package):
    * set the 'publish_odom_tf' parameter to 'true'
* create the `robot2_pose_publisher` node in order to make the robot1 follow the robot2
* modify the launch files so as to spawn the two simple differential robots in the house world and to launch all the necessary nodes for accomplishing the assignment goal:
 * `multi_robot_simulation_launch.py` (contained in the `launch` folder of the newly-created package):
 * `robot_gazebo_spawn_launch.py` (contained in the `launch` folder of the newly-created package):
 * `single_robot_simulation_launch.py` (contained in the `launch` folder of the newly-created package):
