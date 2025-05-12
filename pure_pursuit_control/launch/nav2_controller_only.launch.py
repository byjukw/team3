from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[
                '/home/sibon/ros2_ws/src/pure_pursuit_control/config/vector_pursuit_controller.yaml',
                {'use_sim_time': True}
            ],
            output='screen'
        ),

        # BT Navigator (needed for /navigate_to_pose)
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # Waypoint Follower (optional but safe)
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_controller',
            parameters=[
                {'use_sim_time': True, 'autostart': True,
                 'node_names': ['controller_server', 'bt_navigator', 'waypoint_follower']}
            ],
            output='screen'
        )
    ])

