"""
╔══════════════════════════════════════════════════════════════════════╗
║  PHASE 1 — SLAM MAPPING LAUNCH                                       ║
║                                                                      ║
║  Starts:                                                             ║
║    • RPLidar A1 driver                                               ║
║    • Motor driver (so you can drive with teleop)                     ║
║    • Odometry node                                                   ║
║    • Robot state publisher (TF tree from URDF)                       ║
║    • SLAM Toolbox (async mapping mode)                               ║
║                                                                      ║
║  After launching, in another terminal:                               ║
║    ros2 run teleop_twist_keyboard teleop_twist_keyboard              ║
║  Drive the robot through BOTH rooms slowly.                          ║
║  When done:                                                          ║
║    ros2 run cytron_robot save_map                                    ║
╚══════════════════════════════════════════════════════════════════════╝
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('cytron_robot')

    urdf_file   = os.path.join(pkg, 'urdf', 'robot.urdf')
    slam_config = os.path.join(pkg, 'config', 'slam_params.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        LogInfo(msg='═══ CYTRON ROBOT — SLAM MAPPING MODE ═══'),
        LogInfo(msg='Drive the robot through both rooms, then run: ros2 run cytron_robot save_map'),

        # ── RPLidar A1 ────────────────────────────────────────────────────────
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            parameters=[{
                'serial_port':      '/dev/ttyUSB0',
                'serial_baudrate':  115200,
                'frame_id':         'laser',
                'inverted':         False,
                'angle_compensate': True,
                'scan_mode':        'Standard',
            }],
            output='screen',
        ),

        # ── Robot State Publisher (broadcasts URDF TF tree) ───────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # ── Motor Driver ──────────────────────────────────────────────────────
        Node(
            package='cytron_robot',
            executable='motor_driver',
            name='motor_driver',
            parameters=[{
                'left_pwm_pin':    12,
                'left_dir_pin':    24,
                'right_pwm_pin':   13,
                'right_dir_pin':   25,
                'pwm_frequency':   1000,
                'wheel_radius':    0.033,
                'wheel_baseline':  0.160,
                'max_linear_vel':  0.5,
                'max_angular_vel': 2.0,
                'invert_left':     False,
                'invert_right':    True,
            }],
            output='screen',
        ),

        # ── Odometry Node ─────────────────────────────────────────────────────
        Node(
            package='cytron_robot',
            executable='odometry_node',
            name='odometry_node',
            parameters=[{
                'use_encoders':    False,   # ← Set True if you have encoders wired
                'ticks_per_rev':   20,
                'wheel_radius':    0.033,
                'wheel_baseline':  0.160,
            }],
            output='screen',
        ),

        # ── SLAM Toolbox (Mapping) ─────────────────────────────────────────────
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_config],
            output='screen',
        ),

    ])
