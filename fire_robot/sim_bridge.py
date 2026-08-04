"""
sim_bridge.py

Hardware-free stand-in for fire_bridge.py.

This keeps the SAME odometry math as the real fire_bridge.py
(update_odometry / publish_odom / the tick <-> distance conversion),
so you are testing your friend's actual formulas -- not a fake model.
The only thing swapped out is the source of encoder ticks: instead of
reading them from a serial port connected to the ESP32, we synthesize
them from whatever /cmd_vel command was last received.

Run this instead of fire_bridge when you don't have the ESP32 / motors
connected (e.g. testing in RViz2).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math


class SimBridgeNode(Node):
    def __init__(self):
        super().__init__('sim_bridge')

        # --- Same physical constants as fire_bridge.py ---
        # Keep these in sync with fire_bridge.py so the test is meaningful.
        self.wheel_radius = 0.0215   # meters
        self.wheel_base = 0.152      # meters
        self.ticks_per_rev = 1581.0  # encoder ticks per full rotation

        # Odometry tracking variables (identical to fire_bridge.py)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Last commanded velocity, used to fabricate ticks each tick of the sim clock
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.last_cmd_time = self.get_clock().now()

        # Publishers & subscribers -- same topics as fire_bridge.py
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Simulated "serial read" loop -- fires at 20 Hz like a real encoder stream would
        self.sim_timer = self.create_timer(0.05, self.simulate_encoder_tick)

        self.get_logger().info(
            "sim_bridge running (no hardware) -- publishing fake /odom from /cmd_vel")

    def cmd_vel_callback(self, msg):
        # In fire_bridge.py this writes "L:...,A:...\n" to the serial port.
        # Here we just remember the command so the sim timer can fabricate ticks from it.
        self.last_linear = msg.linear.x
        self.last_angular = msg.angular.z

    def simulate_encoder_tick(self):
        """Fabricate left/right encoder ticks from the last cmd_vel, then feed
        them into the SAME update_odometry() math the real bridge uses."""
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        self.last_cmd_time = now
        if dt <= 0:
            return

        # Differential-drive kinematics: convert body velocity -> per-wheel velocity
        v_left = self.last_linear - (self.last_angular * self.wheel_base / 2.0)
        v_right = self.last_linear + (self.last_angular * self.wheel_base / 2.0)

        distance_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev

        left_ticks = int((v_left * dt) / distance_per_tick)
        right_ticks = int((v_right * dt) / distance_per_tick)

            # REMOVED the early-return on zero ticks — always update/publish,
            # even if ticks are 0, so odom->base_link keeps refreshing at 20Hz.

        self.update_odometry(left_ticks, right_ticks)

    # --- Everything below is copied unchanged from fire_bridge.py ---
    # If your friend edits the real math, mirror the change here too.

    def update_odometry(self, left_ticks, right_ticks):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            return

        distance_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        d_left = left_ticks * distance_per_tick
        d_right = right_ticks * distance_per_tick

        d_center = (d_right + d_left) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        v_linear = d_center / dt
        v_angular = d_theta / dt

        self.x += d_center * math.cos(self.theta + (d_theta / 2.0))
        self.y += d_center * math.sin(self.theta + (d_theta / 2.0))
        self.theta += d_theta

        self.publish_odom(v_linear, v_angular)

    def publish_odom(self, v_linear, v_angular):
        q_z = math.sin(self.theta / 2.0)
        q_w = math.cos(self.theta / 2.0)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = q_z
        t.transform.rotation.w = q_w
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = v_linear
        odom.twist.twist.angular.z = v_angular
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
