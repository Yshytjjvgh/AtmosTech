"""
╔══════════════════════════════════════════════════════════════════════╗
║  PHASE 2 — NAVIGATION LAUNCH                                         ║
║                                                                      ║
║  Starts:                                                             ║
║    • RPLidar A1 driver                                               ║
║    • Motor driver                                                    ║
║    • Odometry node                                                   ║
║    • Robot state publisher                                           ║
║    • Nav2 bringup  (map server + AMCL + planners + controllers)      ║
║    • Room Navigator  (keyboard A/S control)                          ║
║                                                                      ║
║  BEFORE LAUNCHING:                                                   ║
║    1. Confirm map file path in config/nav2_params.yaml               ║
║    2. Set ROOM_A / ROOM_B coordinates in room_navigator.py           ║
║    3. colcon build --packages-select cytron_robot                    ║
║                                                                      ║
║  AFTER LAUNCHING:                                                    ║
║    1. Open RViz (optional, from laptop)                              ║
║    2. Use "2D Pose Estimate" to tell AMCL where the robot starts     ║
║    3. Press A or S in the terminal to navigate                       ║
╚══════════════════════════════════════════════════════════════════════╝
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg      = get_package_share_directory('cytron_robot')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    urdf_file   = os.path.join(pkg, 'urdf',   'robot.urdf')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    map_file    = os.path.join(pkg, 'maps',   'room_map.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        LogInfo(msg='═══ CYTRON ROBOT — NAVIGATION MODE ═══'),
        LogInfo(msg='Press A → Room B  |  Press S → Room A  (in room_navigator terminal)'),

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

        # ── Robot State Publisher ─────────────────────────────────────────────
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
                'use_encoders':   False,   # ← True when encoders are wired
                'ticks_per_rev':  20,
                'wheel_radius':   0.033,
                'wheel_baseline': 0.160,
            }],
            output='screen',
        ),

        # ── Nav2 Bringup (localization + planning + control) ──────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map':           map_file,
                'params_file':   nav2_params,
                'use_sim_time':  'false',
            }.items(),
        ),

        # ── Room Navigator (keyboard A/S) ─────────────────────────────────────
        Node(
            package='cytron_robot',
            executable='room_navigator',
            name='room_navigator',
            output='screen',
            prefix='xterm -e',    # opens in its own terminal window on desktop
                                  # remove 'prefix' if running headless on RPi
        ),

    ])
