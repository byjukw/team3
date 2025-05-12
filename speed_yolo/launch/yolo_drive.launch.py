from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Steering Node (from steering_yolo package)
        #Node(
        #    package='steering_yolo',
        #    executable='yolo_steering_node',
        #    name='yolo_steering_node',
        #    output='screen'
        #),
        
        # Video YOLO Publisher (from speed_yolo)
        Node(
            package='speed_yolo',
            executable='video_yolo_publisher',
            name='video_yolo_publisher',
            output='screen'
        ),

        # PID Tuning Node (from speed_yolo)
        Node(
            package='speed_yolo',
            executable='pid_tuning',
            name='pid_tuning',
            output='screen'
        )
    ])
