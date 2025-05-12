from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(os.getenv('HOME'), 'map.yaml'),
        description='Full path to map yaml file'
    )

    declare_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            os.getenv('HOME'),
            'planning_ws', 'src', 'local_planner', 'param', 'dwb_config.yaml'
        ),
        description='Full path to params yaml file'
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                '/opt/ros/humble/share/nav2_bringup/launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
        }.items()
    )

    return LaunchDescription([
        declare_map_arg,
        declare_params_arg,
        nav2_launch
    ])
