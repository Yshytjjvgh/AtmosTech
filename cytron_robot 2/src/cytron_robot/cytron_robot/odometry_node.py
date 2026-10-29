#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════╗
║  ODOMETRY NODE                                                       ║
║                                                                      ║
║  Reads wheel encoder pulses from GPIO and publishes:                 ║
║    • /odom  (nav_msgs/Odometry)  — required by Nav2 / AMCL          ║
║    • TF transform  odom → base_footprint                             ║
║                                                                      ║
║  ┌─────────── ENCODER GPIO PIN MAP ────────────────────────────┐    ║
║  │                                                              │    ║
║  │  LEFT  encoder channel A  →  GPIO 17  (Physical pin 11)     │    ║
║  │  LEFT  encoder channel B  →  GPIO 27  (Physical pin 13)     │    ║
║  │  RIGHT encoder channel A  →  GPIO 22  (Physical pin 15)     │    ║
║  │  RIGHT encoder channel B  →  GPIO 23  (Physical pin 16)     │    ║
║  │                                                              │    ║
║  │  Encoder VCC  →  3.3V  (Physical pin 1 or 17)               │    ║
║  │  Encoder GND  →  GND   (Physical pin 6)                     │    ║
║  │                                                              │    ║
║  │  NOTE: If your motors have NO encoders, this node           │    ║
║  │  falls back to cmd_vel dead-reckoning odometry.             │    ║
║  │  Set use_encoders: false in the launch file.                │    ║
║  └──────────────────────────────────────────────────────────────┘   ║
║                                                                      ║
║  TUNE THESE FOR YOUR ROBOT:                                         ║
║    TICKS_PER_REV  — pulses per full wheel revolution                ║
║    WHEEL_RADIUS   — in metres                                        ║
║    WHEEL_BASELINE — left-to-right wheel centre distance              ║
╚══════════════════════════════════════════════════════════════════════╝
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math
import time

try:
    import RPi.GPIO as GPIO
    ON_RPI = True
except (ImportError, RuntimeError):
    ON_RPI = False

# ═══════════════════ HARDWARE CONFIGURATION ══════════════════════════════════

# Encoder GPIO pins (BCM)
ENC_LEFT_A  = 17   # Physical pin 11
ENC_LEFT_B  = 27   # Physical pin 13
ENC_RIGHT_A = 22   # Physical pin 15
ENC_RIGHT_B = 23   # Physical pin 16

# Encoder resolution
TICKS_PER_REV = 20   # ← Set to your encoder's pulses-per-revolution
                      #   Common values: 20, 40, 100, 360, 1000
                      #   For motors with 1:48 gearbox + 20-slot encoder: 20*48 = 960

# Robot geometry — keep identical to motor_driver.py and URDF
WHEEL_RADIUS   = 0.033   # metres
WHEEL_BASELINE = 0.160   # metres

# Odometry publish rate
ODOM_RATE_HZ = 20   # Hz

# ═════════════════════════════════════════════════════════════════════════════


class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry_node')

        # Parameters
        self.declare_parameter('use_encoders',   True)
        self.declare_parameter('ticks_per_rev',  TICKS_PER_REV)
        self.declare_parameter('wheel_radius',   WHEEL_RADIUS)
        self.declare_parameter('wheel_baseline', WHEEL_BASELINE)

        self.use_encoders   = self.get_parameter('use_encoders').value
        self.ticks_per_rev  = self.get_parameter('ticks_per_rev').value
        self.wheel_radius   = self.get_parameter('wheel_radius').value
        self.wheel_baseline = self.get_parameter('wheel_baseline').value

        # Robot pose state
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # Encoder tick counters
        self._left_ticks  = 0
        self._right_ticks = 0
        self._last_left_ticks  = 0
        self._last_right_ticks = 0

        # For dead-reckoning fallback from cmd_vel
        self._last_cmd_time  = self.get_clock().now()
        self._cmd_linear     = 0.0
        self._cmd_angular    = 0.0

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # cmd_vel subscriber (used for dead-reckoning when no encoders)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # GPIO encoder setup
        if self.use_encoders and ON_RPI:
            self._setup_encoders()
            self.get_logger().info('Encoder odometry active')
        else:
            self.get_logger().warn(
                'Dead-reckoning odometry active (no encoders). '
                'Accuracy will drift over time. Fine for mapping, '
                'but add encoders for reliable navigation.'
            )

        # Timer
        self._last_time = self.get_clock().now()
        self.create_timer(1.0 / ODOM_RATE_HZ, self._odom_timer_cb)

        self.get_logger().info('Odometry node ready')

    # ── Encoder interrupt setup ───────────────────────────────────────────────
    def _setup_encoders(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in [ENC_LEFT_A, ENC_LEFT_B, ENC_RIGHT_A, ENC_RIGHT_B]:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(ENC_LEFT_A,  GPIO.RISING, callback=self._left_enc_cb)
        GPIO.add_event_detect(ENC_RIGHT_A, GPIO.RISING, callback=self._right_enc_cb)

    def _left_enc_cb(self, channel):
        # Read channel B to determine direction
        if GPIO.input(ENC_LEFT_B):
            self._left_ticks += 1
        else:
            self._left_ticks -= 1

    def _right_enc_cb(self, channel):
        if GPIO.input(ENC_RIGHT_B):
            self._right_ticks -= 1   # mirrored because motor is flipped
        else:
            self._right_ticks += 1

    # ── cmd_vel subscriber (dead-reckoning fallback) ──────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        self._cmd_linear  = msg.linear.x
        self._cmd_angular = msg.angular.z
        self._last_cmd_time = self.get_clock().now()

    # ── Main odometry timer ───────────────────────────────────────────────────
    def _odom_timer_cb(self):
        now = self.get_clock().now()
        dt  = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now

        if dt <= 0:
            return

        if self.use_encoders and ON_RPI:
            # ── Encoder-based odometry ────────────────────────────────────
            d_left_ticks  = self._left_ticks  - self._last_left_ticks
            d_right_ticks = self._right_ticks - self._last_right_ticks
            self._last_left_ticks  = self._left_ticks
            self._last_right_ticks = self._right_ticks

            metres_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
            d_left  = d_left_ticks  * metres_per_tick
            d_right = d_right_ticks * metres_per_tick

            d_centre  = (d_left + d_right) / 2.0
            d_theta   = (d_right - d_left) / self.wheel_baseline

            vx     = d_centre / dt
            v_yaw  = d_theta  / dt
        else:
            # ── Dead-reckoning from cmd_vel ───────────────────────────────
            elapsed = (now - self._last_cmd_time).nanoseconds / 1e9
            if elapsed > 0.5:
                vx    = 0.0
                v_yaw = 0.0
            else:
                vx    = self._cmd_linear
                v_yaw = self._cmd_angular

            d_centre = vx * dt
            d_theta  = v_yaw * dt

        # Integrate pose
        self.theta += d_theta
        self.x     += d_centre * math.cos(self.theta)
        self.y     += d_centre * math.sin(self.theta)

        # Quaternion from yaw
        q = self._yaw_to_quaternion(self.theta)

        stamp = now.to_msg()

        # ── Publish TF: odom → base_footprint ────────────────────────────
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id  = 'base_footprint'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation      = q
        self.tf_broadcaster.sendTransform(tf_msg)

        # ── Publish /odom ─────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'

        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.position.z  = 0.0
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x  = vx
        odom.twist.twist.angular.z = v_yaw

        # Covariance — higher when using dead-reckoning
        cov = 0.01 if (self.use_encoders and ON_RPI) else 0.1
        odom.pose.covariance[0]  = cov    # x
        odom.pose.covariance[7]  = cov    # y
        odom.pose.covariance[35] = cov    # yaw

        self.odom_pub.publish(odom)

        # ── Publish joint states (for robot_state_publisher) ──────────────
        js = JointState()
        js.header.stamp = stamp
        js.name         = ['left_wheel_joint', 'right_wheel_joint']
        js.velocity     = [
            (vx - v_yaw * self.wheel_baseline / 2.0) / self.wheel_radius,
            (vx + v_yaw * self.wheel_baseline / 2.0) / self.wheel_radius,
        ]
        js.position     = [
            (self._left_ticks  / self.ticks_per_rev) * 2 * math.pi if self.use_encoders else 0.0,
            (self._right_ticks / self.ticks_per_rev) * 2 * math.pi if self.use_encoders else 0.0,
        ]
        self.joint_pub.publish(js)

    # ── Helpers ───────────────────────────────────────────────────────────────
    @staticmethod
    def _yaw_to_quaternion(yaw) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def destroy_node(self):
        if self.use_encoders and ON_RPI:
            GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
