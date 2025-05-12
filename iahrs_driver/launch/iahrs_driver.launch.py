#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    iahrs_driver_node = Node(
        package='iahrs_driver', 
        executable='iahrs_driver',
        output='screen',
        parameters=[
            {"m_bSingle_TF_option": True}
        ]
    )
    
    # static_transform_publisher (imu_link를 X축 기준으로 180도 회전)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf_fix',
        arguments=['0', '0', '0', '3.14159', '0', '0', 'base_link', 'imu_link']
    )

    # create and return launch description object
    return LaunchDescription(
        [
            iahrs_driver_node,
            static_tf
        ]
    )