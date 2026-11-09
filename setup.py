from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'slam_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch files, parameters of SLAM Toolbox, Nav2 and AMCL, RViz configs, saved maps
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'maps'),
            glob(os.path.join('maps', '*'))),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonas Main PC',
    maintainer_email='jonas28st@gmail.com',
    description='Day 7: SLAM Toolbox and Nav2 in the loop of slide 8 and the room of day 6',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # name used by "ros2 run" = module:function
            'drive_sim = slam_nav.drive_sim:main',
            'laser_sim = slam_nav.laser_sim:main',
            'robot_viz = slam_nav.robot_viz:main',
            'tour = slam_nav.tour:main',
        ],
    },
)
