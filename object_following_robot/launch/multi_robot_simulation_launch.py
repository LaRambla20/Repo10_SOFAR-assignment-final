# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Example for spawing multiple robots in Gazebo.

This is an example on how to create a launch file for spawning multiple robots into Gazebo
and launch multiple instances of the navigation stack, each controlling one robot.
The robots co-exist on a shared environment and are controlled by independent nav stacks
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node #


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('object_following_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Names and poses of the robots
    robots = [
        {'name': 'robot1', 'x_pose': 0.0, 'y_pose': 0.0, 'z_pose': 0.25}, #
        {'name': 'robot2', 'x_pose': 0.0, 'y_pose': 1.0, 'z_pose': 0.25}]

    # Simulation settings
    world = LaunchConfiguration('world')
    simulator = LaunchConfiguration('simulator')

    # On this example all robots are launched with the same settings
    map_yaml_file = LaunchConfiguration('map')

    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    log_settings = LaunchConfiguration('log_settings', default='true')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(bringup_dir, 'worlds', 'house.world'), #
        description='Full path to world file to load')

    declare_simulator_cmd = DeclareLaunchArgument(
        'simulator',
        default_value='gazebo',
        description='The simulator to use (gazebo or gzserver)')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'house_world.yaml'), #
        description='Full path to map file to load')

    declare_robot1_params_file_cmd = DeclareLaunchArgument( 
        'robot1_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_robot1_params.yaml'), #
        description='Full path to the ROS2 parameters file to use for robot1 launched nodes')

    declare_robot2_params_file_cmd = DeclareLaunchArgument( 
        'robot2_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_robot2_params.yaml'), #
        description='Full path to the ROS2 parameters file to use for robot2 launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='false',
        description='Automatically startup the stacks')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_config_multiple_robots.rviz'), #
        description='Full path to the RVIZ config file to use.')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    # Start Gazebo with plugin providing the robot spawing service
    start_gazebo_cmd = ExecuteProcess(
        cmd=[simulator, '--verbose', '-s', 'libgazebo_ros_init.so',
                                     '-s', 'libgazebo_ros_factory.so', world],
        output='screen')

    # Start robot2_pose_publisher node to publish the transformation between <map> and <robot2/base_footprint> on the goal_pose topic
    start_robot2_pose_publisher_cmd = Node(
        package='object_following_robot',
        executable='robot2_pose_publisher',
        namespace='robot2',
        output='screen',
        prefix='xterm -e',
        remappings= [('/tf', '/robot2/tf'), ('/tf_static', '/robot2/tf_static')]
    )

    # Start teleop_twist_keyboard node
    start_teleop_twist_keyboard_cmd = Node( #
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        namespace='robot2',
        output='screen',
        prefix='xterm -e',
        )

    # Define commands for spawing the robots into Gazebo
    spawn_robots_cmds = []
    for robot in robots:
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch',
                                                           'robot_gazebo_spawn_launch.py')), # SPAWNA I ROBOT IN GAZEBO
                launch_arguments={
                                  'x_pose': TextSubstitution(text=str(robot['x_pose'])), # passa la posa iniziale
                                  'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                                  'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                                  'robot_name': robot['name'], # passa il nome del robot
                                  }.items()))

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        params_file = LaunchConfiguration(f"{robot['name']}_params_file")

        group = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'rviz_launch.py')), # LANCIA RVIZ
                condition=IfCondition(use_rviz),
                launch_arguments={
                                  'namespace': TextSubstitution(text=robot['name']),
                                  'use_namespace': 'True', # si namespace
                                  'rviz_config': rviz_config_file}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup_dir,
                                                           'launch',
                                                           'single_robot_simulation_launch.py')), # LANCIA LA SIMULAZIONE DEI SINGOLI ROBOT
                launch_arguments={'namespace': robot['name'],
                                  'use_namespace': 'True', # si namespace
                                  'map': map_yaml_file, # si mappa, quella che ho definito su
                                  'use_sim_time': 'True', # si usa gazebo simulation time/clock -> per robot state publisher
                                  'params_file': params_file, # si file .yaml del robot considerato
                                  'autostart': autostart, # no autostart
                                  'use_rviz': 'False', # no rviz (l'ho gia lanciato)
                                  'use_simulator': 'False', # no gazebo (ne server ne client)(l'ho gia lanciato)
                                  'headless': 'False', # no gazebo client
                                  'use_robot_state_pub': use_robot_state_pub}.items()), # si robot_state_publisher

            LogInfo(
                condition=IfCondition(log_settings),
                msg=['Launching ', robot['name']]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' map yaml: ', map_yaml_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' params yaml: ', params_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' rviz config file: ', rviz_config_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' using robot state pub: ', use_robot_state_pub]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' autostart: ', autostart])
        ])

        nav_instances_cmds.append(group)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_robot1_params_file_cmd)
    ld.add_action(declare_robot2_params_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)

    # Start robot2_pose_publisher node
    ld.add_action(start_robot2_pose_publisher_cmd)

    # Start teleop twist keyboard node
    ld.add_action(start_teleop_twist_keyboard_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(start_gazebo_cmd)

    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld
