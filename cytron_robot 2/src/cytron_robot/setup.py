from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'cytron_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),  glob('launch/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),  glob('config/*.yaml')),
        # URDF files
        (os.path.join('share', package_name, 'urdf'),    glob('urdf/*.urdf.xacro')),
        (os.path.join('share', package_name, 'urdf'),    glob('urdf/*.urdf')),
        # Maps directory (ships empty; map_saver writes here at runtime)
        (os.path.join('share', package_name, 'maps'),    glob('maps/*') or []),
        # Helper scripts
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sujal',
    maintainer_email='robot@example.com',
    description='Cytron + RPLidar differential-drive robot package',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Motor controller — reads /cmd_vel, drives GPIO pins
            'motor_driver     = cytron_robot.motor_driver:main',
            # Odometry publisher — computes /odom from encoder ticks
            'odometry_node    = cytron_robot.odometry_node:main',
            # Keyboard room navigator (A → Room B, S → Room A)
            'room_navigator   = cytron_robot.room_navigator:main',
            # One-shot map saver utility
            'save_map         = cytron_robot.save_map:main',
        ],
    },
)
