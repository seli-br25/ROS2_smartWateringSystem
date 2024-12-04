import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import InputDevice

class MoisturePublisher(Node):
    def __init__(self):
        super().__init__('moisture_publisher')
        self.publisher_ = self.create_publisher(String, 'soil_moisture', 10)
        self.timer = self.create_timer(1.0, self.publish_moisture_status)
        self.sensor = InputDevice(17)  # GPIO17

    def publish_moisture_status(self):
        status = "moist" if self.sensor.is_active else "dry"
        msg = String()
        msg.data = status
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {status}')

def main(args=None):
    rclpy.init(args=args)
    node = MoisturePublisher()
    rclpy.spin(node)
    rclpy.shutdown()
