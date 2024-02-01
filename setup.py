import os
from glob import glob
from setuptools import setup

package_name = 'sensehat_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'RTIMULib', 'transforms3d', 'sense-hat'],
    zip_safe=True,
    maintainer='Aditya Kamath',
    maintainer_email='adityakamath@live.com',
    description='ROS 2 package for RPi/AstroPi Sense HAT',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensehat_publisher = sensehat_ros.sensehat_publisher:main',
            'sensehat_node = sensehat_ros.sensehat_node:main',
        ],
    },
)
