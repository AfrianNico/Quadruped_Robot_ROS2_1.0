import os
from glob import glob
from setuptools import setup

package_name = 'vision_avoidance_ros2'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Vision node ROS2 (YOLO, RKNN/ONNX, ground-plane distance) untuk penghindaran rintangan quadruped robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = vision_avoidance_ros2.vision_node:main',
        ],
    },
)
