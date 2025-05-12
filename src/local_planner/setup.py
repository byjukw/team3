import os
from glob import glob
from setuptools import setup

package_name = 'local_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # ROS2가 패키지를 인식하도록 하는 필수 파일들
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch 파일, param 파일 등도 설치 경로에 포함
        (os.path.join('share', package_name, 'launch'), glob('local_planner/launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'param'),   glob('param/*.yaml')),
        (os.path.join('share', package_name, 'param'), glob('local_planner/param/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot_maintainer',
    maintainer_email='robot@company.com',
    description='Local Planner Package with TEB for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 만약 python 노드를 실행할 것이 있다면 예: 
            'local_planner_node = local_planner.local_planner:main',
        ],
    },
)
