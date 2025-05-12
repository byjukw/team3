from setuptools import setup

package_name = 'pure_pursuit_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chae',
    maintainer_email='chaewonlim0919@naver.com',
    description='Pure Pursuit Algorithm for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit = pure_pursuit_control.pure_pursuit:main',
            'dummy_path_publisher = pure_pursuit_control.dummy_path_publisher:main',
            'dummy_pose_publisher = pure_pursuit_control.dummy_pose_publisher:main',
            'dummy_tracking_distance_publisher = pure_pursuit_control.dummy_tracking_distance_publisher:main',
            'control_pure_pursuit = pure_pursuit_control.control_pure_pursuit:main',
            'open_loop_follower = pure_pursuit_control.open_loop_path_follower:main',
            'pure_pursuit_node = pure_pursuit_control.pure_pursuit_node:main',
        ],
    },
)

