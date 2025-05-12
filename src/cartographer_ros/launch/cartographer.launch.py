from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Cartographer Node 실행
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', '/home/udh/ros2_ws/src/cartographer_ros/configuration_files',
                '-configuration_basename', 'jetracer.lua'
            ]
        ),

        # Occupancy Grid 생성 노드 (맵 변환)
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            arguments=['-resolution', '0.05'],
            output='screen'
        ),

        
        Node(
            package='cartographer_ros',
            executable='tf_transform_publisher',
            name='tf_transform_publisher',
            output='screen'
        )

    ])
