import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')
        self.subscription = self.create_subscription(
            String,
            'led_control',
            self.listener_callback,
            10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)  # Ersetze `/dev/ttyUSB0` durch deinen Port

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        if msg.data == 'ON':
            self.serial_port.write(b'1')  # Sende '1' an Arduino
        elif msg.data == 'OFF':
            self.serial_port.write(b'0')  # Sende '0' an Arduino

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
