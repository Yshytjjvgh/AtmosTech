#!/usr/bin/env python3
"""
Saves the current SLAM map to the package maps/ directory.
Usage:  ros2 run cytron_robot save_map
"""

import rclpy
from rclpy.node import Node
import subprocess
import os
from ament_index_python.packages import get_package_share_directory


def main(args=None):
    rclpy.init(args=args)
    node = Node('save_map_helper')

    maps_dir = os.path.join(get_package_share_directory('cytron_robot'), 'maps')
    os.makedirs(maps_dir, exist_ok=True)
    map_path = os.path.join(maps_dir, 'room_map')

    node.get_logger().info(f'Saving map to: {map_path}')

    result = subprocess.run(
        ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path],
        capture_output=True, text=True
    )

    if result.returncode == 0:
        node.get_logger().info(f'Map saved! Files:\n  {map_path}.pgm\n  {map_path}.yaml')
        node.get_logger().info(
            'Next step: open navigation.launch.py and confirm map_file path, '
            'then launch navigation.'
        )
    else:
        node.get_logger().error(f'Map save failed:\n{result.stderr}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
