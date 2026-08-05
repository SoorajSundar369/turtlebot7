"""
rotation_test_monitor.py

Same idea as drive_test_monitor.py, but for pure rotation instead of
straight-line driving.

Watches /cmd_vel to detect exactly when angular-only motion starts and
stops (using message timestamps), and tracks /odom orientation the whole
time -- unwrapping the yaw angle so it stays correct even if you spin
more than 360 degrees (a naive quaternion->yaw conversion wraps at +-180
degrees and would otherwise look wrong for a full turn or more).

Usage:
    ros2 run fire_robot rotation_test_monitor

Then, using teleop_twist_keyboard, turn IN PLACE ONLY (no forward/back
key -- rotation only) for a few seconds, then stop. Results print
automatically the moment you stop.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


def yaw_from_quaternion(z, w):
    # Valid for a planar (z-axis only) rotation, which is all this robot uses.
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class RotationTestMonitor(Node):
    def __init__(self):
        super().__init__('rotation_test_monitor')

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.driving = False
        self.turn_start_time = None
        self.commanded_angular = 0.0
        self.commanded_linear = 0.0

        # Unwrapped cumulative yaw tracking
        self.last_yaw = None
        self.unwrapped_yaw = 0.0
        self.yaw_at_turn_start = None

        self.get_logger().info(
            "rotation_test_monitor ready. Turn IN PLACE ONLY with "
            "teleop_twist_keyboard (no forward/back), then stop -- "
            "results print automatically."
        )

    def odom_callback(self, msg):
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        yaw = yaw_from_quaternion(z, w)

        if self.last_yaw is not None:
            delta = yaw - self.last_yaw
            # wrap delta into [-pi, pi] so crossing the +-180 boundary
            # doesn't create a fake huge jump
            while delta > math.pi:
                delta -= 2 * math.pi
            while delta < -math.pi:
                delta += 2 * math.pi
            self.unwrapped_yaw += delta

        self.last_yaw = yaw

    def cmd_vel_callback(self, msg):
        is_turning = abs(msg.angular.z) > 1e-6
        is_moving_linear = abs(msg.linear.x) > 1e-6
        now = self.get_clock().now()

        if is_turning and not self.driving:
            self.driving = True
            self.turn_start_time = now
            self.commanded_angular = msg.angular.z
            self.commanded_linear = msg.linear.x
            self.yaw_at_turn_start = self.unwrapped_yaw
            self.get_logger().info(
                f"Turn START detected (angular={msg.angular.z:.2f} rad/s, "
                f"linear={msg.linear.x:.2f} m/s)"
            )

        elif not is_turning and self.driving:
            self.driving = False
            turn_end_time = now

            duration = (turn_end_time - self.turn_start_time).nanoseconds / 1e9
            actual_rotation = self.unwrapped_yaw - self.yaw_at_turn_start
            expected_rotation = self.commanded_angular * duration

            if abs(self.commanded_linear) > 1e-6:
                self.get_logger().warn(
                    "Linear velocity was non-zero during this turn -- "
                    "this was an arc, not a pure rotation. Repeat with "
                    "turn-only keys (no forward/back)."
                )

            if abs(expected_rotation) > 1e-9:
                error_pct = 100.0 * (actual_rotation - expected_rotation) / expected_rotation
            else:
                error_pct = float('nan')

            self.get_logger().info("=" * 50)
            self.get_logger().info("ROTATION TEST RESULT")
            self.get_logger().info(f"  Measured duration:    {duration:.3f} s")
            self.get_logger().info(f"  Commanded ang. vel:   {self.commanded_angular:.3f} rad/s")
            self.get_logger().info(
                f"  Expected rotation:    {expected_rotation:.4f} rad "
                f"({math.degrees(expected_rotation):.1f} deg)"
            )
            self.get_logger().info(
                f"  Actual rotation:      {actual_rotation:.4f} rad "
                f"({math.degrees(actual_rotation):.1f} deg)"
            )
            self.get_logger().info(f"  Error:                {error_pct:.1f} %")
            self.get_logger().info("=" * 50)


def main(args=None):
    rclpy.init(args=args)
    node = RotationTestMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
