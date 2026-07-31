import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class ESP32BridgeNode(Node):
    def __init__(self):
        super().__init__('fire_bridge')
        
        # ⚠️ Make sure this matches your ESP32 port! (Usually /dev/ttyUSB0)
        self.port = '/dev/ttyUSB0' 
        self.baud_rate = 115200
        
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=0.1)
            time.sleep(2.0)
            self.get_logger().info(f"✅ Connected to ESP32 on {self.port}")
        except Exception as e:
            self.get_logger().error(f"❌ Could not open serial port {self.port}: {e}")
            return

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        command = f"L:{msg.linear.x:.2f},A:{msg.angular.z:.2f}\n"
        try:
            self.ser.write(command.encode('utf-8'))
            self.get_logger().info(f"Sent: {command.strip()}")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.write("L:0.00,A:0.00\n".encode('utf-8'))
            node.ser.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
