from setuptools import find_packages, setup
from glob import glob

package_name = 'steering_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}', glob('steering_yolo/*.pt')),  # ← 모델 포함
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chae',
    maintainer_email='chaewonlim0919@naver.com',
    description='YOLO 기반 실시간 조향각 퍼블리셔 패키지',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_steering_node = steering_yolo.yolo_steering_node:main'
        ],
    },
)

