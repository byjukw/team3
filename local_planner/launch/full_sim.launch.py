#!/usr/bin/env python3
"""
full_sim.launch.py
Gazebo‑TurtleBot3 시뮬 + SLAM/AMCL(localization) + RViz2 + Nav2 bringup
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ---------- 인자 ----------
    map_yaml = '/home/mcity/planning_ws/src/local_planner/maps/map.yaml'
    params_yaml = '/home/mcity/planning_ws/src/local_planner/param/dwb_config.yaml'

    # ---------- include: Gazebo ----------
    turtlebot3_world_launch = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch', 'turtlebot3_world.launch.py')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(turtlebot3_world_launch))

    # ---------- include: localization ----------
    localization_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch', 'localization_launch.py')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': 'true',
            'map_subscribe_transient_local': 'true'
        }.items())

    # ---------- include: nav2 bringup ----------
    bringup_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch', 'bringup_launch.py')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': params_yaml
        }.items())

    # ---------- node: RViz2 ----------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        parameters=[{'use_sim_time': True}]
        # 필요하면 `arguments=['-d', '<rviz config>']` 추가
    )

    # ---------- LaunchDescription ----------
    ld = LaunchDescription([
        gazebo,                       # ① Gazebo
        TimerAction(period=3.0, action=localization),   # ② localization (약간 지연)
        TimerAction(period=5.0, action=nav2_bringup),   # ③ Nav2 bringup (더 지연)
        TimerAction(period=6.0, action=rviz)            # ④ RViz2
    ])
    return ld
