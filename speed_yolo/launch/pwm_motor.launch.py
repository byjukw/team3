from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='speed_yolo',
            executable='video_yolo_publisher',
            name='video_yolo_publisher',
            output='screen'
        ),
        Node(
            package='speed_yolo',
            executable='motor_controller',  
            name='motor_controller',
            output='screen'
        ),
    ])

