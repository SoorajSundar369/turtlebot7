import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import time
import math
import threading

class ESP32BridgeNode(Node):
    def __init__(self):
        super().__init__('fire_bridge')

        self.port = '/dev/ttyUSB1'
        self.baud_rate = 115200

        # --- PHYSICAL ROBOT MEASUREMENTS (WE NEED TO TWEAK THESE) ---
        self.wheel_radius = 0.0215
        self.wheel_base = 0.152
        self.ticks_per_rev = 1581.0

        # Odometry Tracking Variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # --- Publishers & Subscribers created FIRST, unconditionally ---
        # This way, even if the serial connection below fails, /cmd_vel is
        # still subscribed and /odom still exists -- the node just won't be
        # able to talk to the ESP32.
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Serial connection is now optional ---
        self.ser = None
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=0.1)
            time.sleep(2.0)
            self.get_logger().info(f"✅ Connected to ESP32 on {self.port}")
        except Exception as e:
            self.get_logger().error(
                f"❌ Could not open serial port {self.port}: {e}. "
                f"Node will keep running but motors/encoders are disconnected."
            )

        # Only start the read thread if serial actually connected
        if self.ser is not None:
            self.read_thread = threading.Thread(target=self.read_serial)
            self.read_thread.daemon = True
            self.read_thread.start()

    def cmd_vel_callback(self, msg):
        if self.ser is None:
            return  # no hardware connected, nothing to write to
        command = f"L:{msg.linear.x:.2f},A:{msg.angular.z:.2f}\n"
        try:
            self.ser.write(command.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def read_serial(self):
        while rclpy.ok() and self.ser is not None and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line.startswith("E:"):
                    parts = line[2:].split(',')
                    if len(parts) == 2:
                        self.update_odometry(int(parts[0]), int(parts[1]))
            except Exception:
                pass

    def update_odometry(self, left_ticks, right_ticks):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0: return

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
    node = ESP32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.ser is not None and node.ser.is_open:
            node.ser.write("L:0.00,A:0.00\n".encode('utf-8'))
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
