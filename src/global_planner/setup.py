from setuptools import find_packages, setup

package_name = 'global_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyun',
    maintainer_email='hyun@todo.todo',
    description='Global path planner using D* Lite for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_planner = global_planner.global_planner:main',
            'global_planner_scenario = global_planner.global_planner_scenario:main',
        ],
    },
)
