#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Config #
    useSimTime = LaunchConfiguration('use_sim_time', default='true')
    worldName = LaunchConfiguration('world', default='turtlebot3_world')
    launchRViz = LaunchConfiguration('rviz', default='true')

    # Package Directories #
    pkgTurtlebot3Gazebo = FindPackageShare('turtlebot3_gazebo')
    pkgTurtlebot3Bringup= FindPackageShare('turtlebot3_bringup')
    
    # Declare Launch Arguments #
    declareUseSimTimeCMD = DeclareLaunchArgument(
                'use_sim_time', 
                default_value = 'true',
                description = 'Use simulation (Gazebo) clock if true'
            )

    declareWorldCMD = DeclareLaunchArgument(
                'world',
                default_value = 'turtlebot3_world',
                description='name of world model'
            )

    declareRVizCMD = DeclareLaunchArgument(
                'rviz',
                default_value = 'true',
                description = 'Launch RViz if true'
            )

    # Launch Gazebo #
    gazeboLaunch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([pkgTurtlebot3Gazebo, 'launch', 'turtlebot3_world.launch.py'])
                    ]),
                    launch_arguments = {'use_sim_time': useSimTime}.items()
                )

    # Launch RViz #
    rvizLaunch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([pkgTurtlebot3Bringup, 'launch', 'rviz2.launch.py'])
                    ]),
                    condition = IfCondition(launchRViz),
                    launch_arguments = {'use_sim_time':useSimTime}.items()
                )

    # Custom DWA #
    # dwaPlannerNode = Node(
    #       package = 'custom_dwa',
    #        executable = 'dwa',
    #        name = 'custom_dwa',
    #        output = 'screen',
    #        parameters = [{ 'use_sim_time', useSimTime, }],
    #        remappings = [
    #            ('/goal_pose', '/goal_pose'),
    #            ('/cmd_vel', '/cmd_vel'),
    #            ('/odom', '/odom'),
    #            ('/scan', '/scan'),
    #        ]
    #    )

    # Create Launch Description #
    ld = LaunchDescription()

    # Add Launch Arguments #
    ld.add_action(declareUseSimTimeCMD)
    ld.add_action(declareWorldCMD)
    ld.add_action(declareRVizCMD)

    # Add Nodes #
    ld.add_action(gazeboLaunch)
    ld.add_action(rvizLaunch)
    # ld.add_action(dwaPlannerNode)

    return ld

