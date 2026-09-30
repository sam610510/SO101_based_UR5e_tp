import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'so100_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sam',
    maintainer_email='sam610510@gmail.com',
    description='SO101 leader -> UR5e follower ROS2 teleoperation node',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ur5e_teleop_node = so100_control.so101_ur5e_teleop_node:main',
        ],
    },
)
