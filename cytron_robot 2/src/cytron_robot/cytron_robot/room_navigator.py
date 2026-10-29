#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════╗
║  ROOM NAVIGATOR NODE                                                 ║
║                                                                      ║
║  Keyboard control for autonomous room navigation.                    ║
║                                                                      ║
║  Press A  →  Robot navigates to Room B                              ║
║  Press S  →  Robot navigates to Room A                              ║
║  Press C  →  Cancel current navigation goal                         ║
║  Press Q  →  Quit                                                    ║
║                                                                      ║
║  HOW TO SET ROOM COORDINATES:                                        ║
║    1. Build and save your map (Phase 1)                              ║
║    2. Launch navigation (Phase 2)                                    ║
║    3. Open RViz → click "2D Pose Estimate" → click centre of Room A ║
║    4. Note the x, y printed in terminal                              ║
║    5. Edit ROOM_A and ROOM_B coordinates below                       ║
║    6. Rebuild package:  colcon build --packages-select cytron_robot  ║
╚══════════════════════════════════════════════════════════════════════╝
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import sys
import termios
import tty
import threading
import math


# ═══════════════════ ROOM COORDINATES ════════════════════════════════════════
#
#  Set these after mapping.  Values are in the MAP frame (metres).
#  orientation_z / orientation_w = quaternion for the robot's heading.
#
#  Quick heading reference (facing direction):
#    Facing +X axis (East) :  z=0.000, w=1.000
#    Facing +Y axis (North):  z=0.707, w=0.707
#    Facing -X axis (West) :  z=1.000, w=0.000
#    Facing -Y axis (South):  z=-0.707, w=0.707
#
ROOM_A = {
    'name': 'Room A',
    'x': 1.0,     # ← CHANGE THIS after mapping
    'y': 0.5,     # ← CHANGE THIS after mapping
    'z': 0.0,
    'w': 1.0,
}

ROOM_B = {
    'name': 'Room B',
    'x': 4.5,     # ← CHANGE THIS after mapping
    'y': 2.0,     # ← CHANGE THIS after mapping
    'z': 0.0,
    'w': 1.0,
}
# ═════════════════════════════════════════════════════════════════════════════


class RoomNavigator(Node):

    def __init__(self):
        super().__init__('room_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._current_goal_handle: ClientGoalHandle = None
        self._navigating = False

        self.get_logger().info('Room Navigator ready — waiting for Nav2...')
        self._action_client.wait_for_server(timeout_sec=30.0)
        self.get_logger().info('Nav2 connected!')

    # ── Send a navigation goal ────────────────────────────────────────────────
    def navigate_to(self, room: dict):
        if self._navigating:
            self.get_logger().info('Cancelling previous goal...')
            self.cancel_goal()

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()

        pose.pose.position.x    = room['x']
        pose.pose.position.y    = room['y']
        pose.pose.position.z    = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = room['z']
        pose.pose.orientation.w = room['w']

        goal.pose = pose

        self.get_logger().info(
            f"Navigating to {room['name']} "
            f"(x={room['x']:.2f}, y={room['y']:.2f})"
        )
        self._navigating = True

        send_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb
        )
        send_future.add_done_callback(self._goal_response_cb)

    # ── Cancel current goal ───────────────────────────────────────────────────
    def cancel_goal(self):
        if self._current_goal_handle is not None:
            cancel_future = self._current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda f: self.get_logger().info('Goal cancelled')
            )
            self._navigating = False

    # ── Action callbacks ──────────────────────────────────────────────────────
    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED by Nav2')
            self._navigating = False
            return

        self._current_goal_handle = goal_handle
        self.get_logger().info('Goal accepted — robot is moving...')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        status = result.status
        self._navigating = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✓ Goal reached successfully!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal was cancelled')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(
                'Goal ABORTED — robot may be stuck or path not found. '
                'Try giving a new initial pose in RViz.'
            )
        else:
            self.get_logger().warn(f'Goal ended with status: {status}')

    def _feedback_cb(self, feedback_msg):
        fb  = feedback_msg.feedback
        dist = fb.distance_remaining
        self.get_logger().info(f'Distance remaining: {dist:.2f} m', throttle_duration_sec=2.0)


# ── Keyboard input loop (runs in its own thread) ──────────────────────────────
def _get_key() -> str:
    """Read one keypress without waiting for Enter."""
    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _keyboard_loop(navigator: RoomNavigator):
    print('\n' + '═' * 52)
    print('  ROOM NAVIGATOR CONTROLS')
    print('  A  →  Go to Room B')
    print('  S  →  Go to Room A')
    print('  C  →  Cancel navigation')
    print('  Q  →  Quit')
    print('═' * 52 + '\n')

    while rclpy.ok():
        key = _get_key().lower()

        if key == 'a':
            navigator.navigate_to(ROOM_B)
        elif key == 's':
            navigator.navigate_to(ROOM_A)
        elif key == 'c':
            navigator.cancel_goal()
        elif key == 'q':
            print('\nQuitting room navigator...')
            rclpy.shutdown()
            break
        elif key == '\x03':   # Ctrl+C
            rclpy.shutdown()
            break


def main(args=None):
    rclpy.init(args=args)
    navigator = RoomNavigator()

    kb_thread = threading.Thread(
        target=_keyboard_loop,
        args=(navigator,),
        daemon=True
    )
    kb_thread.start()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
