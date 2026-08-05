"""
drive_test_monitor.py

Automatically measures odometry accuracy without manual stopwatch timing.

Watches /cmd_vel to detect exactly when you START driving (first non-zero
command) and STOP driving (command returns to zero), using message
timestamps -- not wall-clock guesses. Records /odom position at both
moments and reports:

    - measured drive duration (from cmd_vel timestamps)
    - commanded speed (from the cmd_vel messages themselves)
    - expected distance = commanded_speed * measured_duration
    - actual distance = straight-line distance between odom readings
    - percentage error between the two

Usage:
    ros2 run fire_robot drive_test_monitor
    (or: python3 drive_test_monitor.py  if rclpy is sourced)

Then just drive normally with teleop_twist_keyboard -- press forward,
hold/tap for a few seconds, then stop. The moment you stop, this node
prints the comparison automatically. Works best with STRAIGHT-LINE
driving only (no turning) for a clean linear-distance comparison.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class DriveTestMonitor(Node):
    def __init__(self):
        super().__init__('drive_test_monitor')

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.driving = False
        self.drive_start_time = None
        self.drive_start_pos = None
        self.commanded_linear = 0.0
        self.commanded_angular = 0.0
        self.latest_odom_pos = None

        self.get_logger().info(
            "drive_test_monitor ready. Drive straight with teleop_twist_keyboard, "
            "then stop -- results print automatically."
        )

    def odom_callback(self, msg):
        self.latest_odom_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    def cmd_vel_callback(self, msg):
        is_moving = abs(msg.linear.x) > 1e-6 or abs(msg.angular.z) > 1e-6
        now = self.get_clock().now()

        if is_moving and not self.driving:
            # Driving just started
            self.driving = True
            self.drive_start_time = now
            self.drive_start_pos = self.latest_odom_pos
            self.commanded_linear = msg.linear.x
            self.commanded_angular = msg.angular.z
            self.get_logger().info(
                f"Drive START detected (linear={msg.linear.x:.2f} m/s, "
                f"angular={msg.angular.z:.2f} rad/s)"
            )

        elif not is_moving and self.driving:
            # Driving just stopped -- compute and report
            self.driving = False
            drive_end_time = now
            drive_end_pos = self.latest_odom_pos

            if self.drive_start_pos is None or drive_end_pos is None:
                self.get_logger().warn("Missing odom data, can't compute result.")
                return

            duration = (drive_end_time - self.drive_start_time).nanoseconds / 1e9

            dx = drive_end_pos[0] - self.drive_start_pos[0]
            dy = drive_end_pos[1] - self.drive_start_pos[1]
            actual_distance = math.sqrt(dx * dx + dy * dy)

            if abs(self.commanded_angular) > 1e-6:
                self.get_logger().warn(
                    "Angular velocity was non-zero during this drive -- "
                    "straight-line distance comparison is NOT valid for "
                    "turning motion. Repeat with forward-only driving."
                )

            expected_distance = abs(self.commanded_linear) * duration
            if expected_distance > 1e-9:
                error_pct = 100.0 * (actual_distance - expected_distance) / expected_distance
            else:
                error_pct = float('nan')

            self.get_logger().info("=" * 50)
            self.get_logger().info("DRIVE TEST RESULT")
            self.get_logger().info(f"  Measured duration:   {duration:.3f} s")
            self.get_logger().info(f"  Commanded speed:     {self.commanded_linear:.3f} m/s")
            self.get_logger().info(f"  Expected distance:   {expected_distance:.4f} m")
            self.get_logger().info(f"  Actual distance:     {actual_distance:.4f} m")
            self.get_logger().info(f"  Error:               {error_pct:.1f} %")
            self.get_logger().info("=" * 50)


def main(args=None):
    rclpy.init(args=args)
    node = DriveTestMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
