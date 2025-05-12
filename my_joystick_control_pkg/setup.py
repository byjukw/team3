from setuptools import find_packages, setup

package_name = 'my_joystick_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/joystick_control.launch.py']),
        ('share/' + package_name + '/config', ['config/teleop_twist_joy.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chae',
    maintainer_email='chaewonlim0919@naver.com',
    description='Joystick control package for manual driving',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_to_steering.py = my_joystick_control_pkg.joystick_to_steering:main',
        ],
    },
)

