from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('teleop_twist_joy'),
                'launch',
                'teleop-launch.py'
            )
        )
    )

    return LaunchDescription([
        # joy_node는 teleop-launch.py 내부에 이미 포함돼 있으므로, 따로 실행할 필요 없음!
        
        teleop_launch,

        # joystick_to_steering.py 실행
        Node(
            package='my_joystick_control_pkg',
            executable='joystick_to_steering.py',
            name='joystick_to_steering_node',
            output='screen'
        )
    ])

