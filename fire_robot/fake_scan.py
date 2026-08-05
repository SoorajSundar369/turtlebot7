"""
fake_scan.py

Publishes a synthetic /scan (sensor_msgs/LaserScan) message so you can
verify RViz's LaserScan display and SLAM Toolbox are wired correctly
(correct topic name, correct frame_id, display settings) WITHOUT a real
RPLidar attached.

This is NOT real distance data -- it just publishes a slowly-rotating
"room" of fake obstacles so you can visually confirm rays are appearing
in RViz. Once the real RPLidar is connected, remove this node and use
the real rplidar_composition node instead.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class FakeScanNode(Node):
    def __init__(self):
        super().__init__('fake_scan')

        self.frame_id = 'laser_frame'   # must match your URDF's laser link name
        self.num_readings = 360         # one reading per degree
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.range_min = 0.15
        self.range_max = 8.0

        self.phase = 0.0  # used to slowly animate the fake obstacles

        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)  # 10 Hz, typical LIDAR rate

        self.get_logger().info(
            "fake_scan running (no hardware) -- publishing synthetic /scan on frame '%s'"
            % self.frame_id
        )

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = (self.angle_max - self.angle_min) / self.num_readings
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        ranges = []
        for i in range(self.num_readings):
            angle = self.angle_min + i * msg.angle_increment
            # Fake a rectangular "room" a few meters across, slowly rotating
            # so it's visually obvious in RViz that data is live, not static.
            a = angle + self.phase
            room_half_size = 3.0
            # distance to a simple square boundary from the origin, along angle a
            cos_a, sin_a = math.cos(a), math.sin(a)
            if abs(cos_a) > abs(sin_a):
                dist = abs(room_half_size / cos_a)
            else:
                dist = abs(room_half_size / sin_a)
            dist = min(dist, self.range_max)
            ranges.append(dist)

        msg.ranges = ranges
        msg.intensities = []

        self.publisher.publish(msg)
        self.phase += 0.01  # slow rotation each tick, purely visual


def main(args=None):
    rclpy.init(args=args)
    node = FakeScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
