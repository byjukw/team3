from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Cartographer Node (Localization)
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', '/home/hyun/planning_ws/src/cartographer_ros/configuration_files',
                #lua 파일 불러오는 경로
                '-configuration_basename', 'carto_local.lua',
                
                # '-configuration_basename', 'a3.yaml',

                # 찐 맵
                # '--load_state_filename', '/home/hyun/planning_ws/src/cartographer_ros/map/b3.pbstream',
                # 공별
                '--load_state_filename', '/home/hyun/planning_ws/src/cartographer_ros/map/b131.pbstream',
                

                #저장된 map 불러오는 경로
                '--load_frozen_state', 'true'
            ]
        ),

        # Occupancy Grid 노드는 optional (원하면 주석 처리 가능)
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            arguments=['-resolution', '0.05'],
            output='screen'
        ),

        # base_footprint → base_link static TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='footprint_to_base',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_footprint', 'base_link']
        )
    ])
