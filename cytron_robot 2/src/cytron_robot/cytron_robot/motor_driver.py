#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════╗
║  CYTRON MOTOR DRIVER NODE                                           ║
║                                                                      ║
║  Subscribes to /cmd_vel (geometry_msgs/Twist) and drives two        ║
║  motors via Cytron MDD3A / MDD10A using PWM + DIR signals.          ║
║                                                                      ║
║  ┌─────────────────── GPIO PIN MAP ────────────────────────────┐    ║
║  │                                                              │    ║
║  │  Cytron Driver → RPi GPIO (BCM numbering)                   │    ║
║  │                                                              │    ║
║  │  LEFT  MOTOR:                                                │    ║
║  │    PWM1  (speed)     →  GPIO 12  (Hardware PWM0)            │    ║
║  │    DIR1  (direction) →  GPIO 24                             │    ║
║  │                                                              │    ║
║  │  RIGHT MOTOR:                                                │    ║
║  │    PWM2  (speed)     →  GPIO 13  (Hardware PWM1)            │    ║
║  │    DIR2  (direction) →  GPIO 25                             │    ║
║  │                                                              │    ║
║  │  GND   →  Any RPi GND pin (e.g. Pin 6, 9, 14, 20, 25...)   │    ║
║  │  VCC   →  5V RPi pin OR external 5V supply                  │    ║
║  │  VMOT  →  Battery+ (7.4V LiPo or 9–12V supply)             │    ║
║  │                                                              │    ║
║  │  RPi Header reference:                                       │    ║
║  │    GPIO12 = Physical Pin 32                                  │    ║
║  │    GPIO13 = Physical Pin 33                                  │    ║
║  │    GPIO24 = Physical Pin 18                                  │    ║
║  │    GPIO25 = Physical Pin 22                                  │    ║
║  └──────────────────────────────────────────────────────────────┘   ║
║                                                                      ║
║  Cytron MDD3A / MDD10A channel mapping:                             ║
║    Channel A → Left  motor  (PWM1 + DIR1)                           ║
║    Channel B → Right motor  (PWM2 + DIR2)                           ║
║                                                                      ║
║  ROBOT PARAMETERS (tune to your hardware):                          ║
║    WHEEL_RADIUS   = 0.033  m                                        ║
║    WHEEL_BASELINE = 0.160  m  (centre-to-centre)                    ║
║    MAX_LINEAR_VEL = 0.5    m/s                                      ║
║    MAX_ANGULAR_VEL= 2.0    rad/s                                     ║
╚══════════════════════════════════════════════════════════════════════╝
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

# ── GPIO import with simulation fallback ─────────────────────────────────────
try:
    import RPi.GPIO as GPIO
    ON_RPI = True
except (ImportError, RuntimeError):
    ON_RPI = False
    print("[motor_driver] WARNING: RPi.GPIO not found — running in simulation mode")

# ═══════════════════ HARDWARE CONFIGURATION ══════════════════════════════════
# Change these if you wire the board differently.

# BCM pin numbers
LEFT_PWM_PIN  = 12   # Physical pin 32  — PWM0 (hardware PWM)
LEFT_DIR_PIN  = 24   # Physical pin 18
RIGHT_PWM_PIN = 13   # Physical pin 33  — PWM1 (hardware PWM)
RIGHT_DIR_PIN = 25   # Physical pin 22

PWM_FREQUENCY = 1000  # Hz  (1 kHz works well for Cytron MDD3A/MDD10A)

# Robot geometry — must match URDF and odometry node
WHEEL_RADIUS   = 0.033   # metres (measure your actual wheel)
WHEEL_BASELINE = 0.160   # metres (left wheel centre → right wheel centre)

# Velocity clamps
MAX_LINEAR_VEL  = 0.50   # m/s
MAX_ANGULAR_VEL = 2.00   # rad/s

# cmd_vel watchdog — stop motors if no message received within this time
CMD_VEL_TIMEOUT = 0.5    # seconds

# Motor direction inversion (set True if motor runs backwards)
INVERT_LEFT  = False
INVERT_RIGHT = True   # Right motor is usually mirrored on diff-drive robots


# ═════════════════════════════════════════════════════════════════════════════

class MotorDriverNode(Node):

    def __init__(self):
        super().__init__('motor_driver')

        # ── Parameters (overridable from launch file / CLI) ───────────────
        self.declare_parameter('left_pwm_pin',   LEFT_PWM_PIN)
        self.declare_parameter('left_dir_pin',   LEFT_DIR_PIN)
        self.declare_parameter('right_pwm_pin',  RIGHT_PWM_PIN)
        self.declare_parameter('right_dir_pin',  RIGHT_DIR_PIN)
        self.declare_parameter('pwm_frequency',  PWM_FREQUENCY)
        self.declare_parameter('wheel_radius',   WHEEL_RADIUS)
        self.declare_parameter('wheel_baseline', WHEEL_BASELINE)
        self.declare_parameter('max_linear_vel', MAX_LINEAR_VEL)
        self.declare_parameter('max_angular_vel',MAX_ANGULAR_VEL)
        self.declare_parameter('invert_left',    INVERT_LEFT)
        self.declare_parameter('invert_right',   INVERT_RIGHT)

        lp  = self.get_parameter('left_pwm_pin').value
        ld  = self.get_parameter('left_dir_pin').value
        rp  = self.get_parameter('right_pwm_pin').value
        rd  = self.get_parameter('right_dir_pin').value
        freq= self.get_parameter('pwm_frequency').value

        self.wheel_radius   = self.get_parameter('wheel_radius').value
        self.wheel_baseline = self.get_parameter('wheel_baseline').value
        self.max_lin        = self.get_parameter('max_linear_vel').value
        self.max_ang        = self.get_parameter('max_angular_vel').value
        self.invert_left    = self.get_parameter('invert_left').value
        self.invert_right   = self.get_parameter('invert_right').value

        # ── GPIO setup ────────────────────────────────────────────────────
        if ON_RPI:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)

            GPIO.setup(lp, GPIO.OUT)
            GPIO.setup(ld, GPIO.OUT)
            GPIO.setup(rp, GPIO.OUT)
            GPIO.setup(rd, GPIO.OUT)

            self.left_pwm  = GPIO.PWM(lp, freq)
            self.right_pwm = GPIO.PWM(rp, freq)
            self.left_pwm.start(0)
            self.right_pwm.start(0)

            self._left_dir_pin  = ld
            self._right_dir_pin = rd
            self.get_logger().info(
                f'GPIO initialised — L: PWM={lp}/DIR={ld}  R: PWM={rp}/DIR={rd}  @{freq}Hz'
            )
        else:
            self.get_logger().warn('Simulation mode: GPIO calls skipped')

        # ── ROS subscriber ────────────────────────────────────────────────
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10
        )

        # ── Watchdog timer — stops motors if cmd_vel goes silent ──────────
        self._last_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self._watchdog_cb)   # check every 100 ms

        self.get_logger().info('Motor driver node ready — listening on /cmd_vel')

    # ── cmd_vel callback ──────────────────────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()

        linear  = self._clamp(msg.linear.x,  -self.max_lin, self.max_lin)
        angular = self._clamp(msg.angular.z, -self.max_ang, self.max_ang)

        # Differential drive inverse kinematics
        # v_left  = linear - (angular * baseline / 2)
        # v_right = linear + (angular * baseline / 2)
        v_left  = linear - (angular * self.wheel_baseline / 2.0)
        v_right = linear + (angular * self.wheel_baseline / 2.0)

        # Convert m/s to PWM duty cycle (0–100)
        # Maximum wheel speed when running at 100% duty
        max_wheel_speed = self.max_lin + (self.max_ang * self.wheel_baseline / 2.0)
        max_wheel_speed = max(max_wheel_speed, 0.001)  # avoid div/0

        duty_left  = (abs(v_left)  / max_wheel_speed) * 100.0
        duty_right = (abs(v_right) / max_wheel_speed) * 100.0

        # Clamp to [0, 100]
        duty_left  = self._clamp(duty_left,  0.0, 100.0)
        duty_right = self._clamp(duty_right, 0.0, 100.0)

        # Direction: True = forward
        dir_left  = v_left  >= 0
        dir_right = v_right >= 0

        if self.invert_left:
            dir_left = not dir_left
        if self.invert_right:
            dir_right = not dir_right

        self._set_motor(self.left_pwm  if ON_RPI else None,
                        self._left_dir_pin  if ON_RPI else 0,
                        duty_left,  dir_left,  'L')
        self._set_motor(self.right_pwm if ON_RPI else None,
                        self._right_dir_pin if ON_RPI else 0,
                        duty_right, dir_right, 'R')

        self.get_logger().debug(
            f'cmd_vel lin={linear:.2f} ang={angular:.2f} | '
            f'L: {duty_left:.0f}% {"FWD" if dir_left else "REV"}  '
            f'R: {duty_right:.0f}% {"FWD" if dir_right else "REV"}'
        )

    # ── Watchdog ──────────────────────────────────────────────────────────────
    def _watchdog_cb(self):
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > CMD_VEL_TIMEOUT:
            self._stop_motors()

    # ── Hardware helpers ──────────────────────────────────────────────────────
    def _set_motor(self, pwm_obj, dir_pin, duty, forward, label):
        if ON_RPI:
            GPIO.output(dir_pin, GPIO.HIGH if forward else GPIO.LOW)
            pwm_obj.ChangeDutyCycle(duty)
        else:
            self.get_logger().debug(
                f'[SIM] Motor {label}: duty={duty:.1f}% dir={"FWD" if forward else "REV"}'
            )

    def _stop_motors(self):
        if ON_RPI:
            self.left_pwm.ChangeDutyCycle(0)
            self.right_pwm.ChangeDutyCycle(0)

    # ── Cleanup ───────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._stop_motors()
        if ON_RPI:
            self.left_pwm.stop()
            self.right_pwm.stop()
            GPIO.cleanup()
            self.get_logger().info('GPIO cleaned up')
        super().destroy_node()

    @staticmethod
    def _clamp(value, lo, hi):
        return max(lo, min(hi, value))


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
