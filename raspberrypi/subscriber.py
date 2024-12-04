import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import OutputDevice

class RelayController(Node):
    def __init__(self):
        super().__init__('relay_controller')
        self.subscription = self.create_subscription(
            String,
            'soil_moisture',
            self.control_relay,
            10
        )
        self.relay = OutputDevice(27, active_high=True, initial_value=False)  # GPIO27

    def control_relay(self, msg):
        if msg.data == "dry":
            self.relay.on()  # Turn on relay
            self.get_logger().info('Relay Activated: Soil is dry.')
        else:
            self.relay.off()  # Turn off relay
            self.get_logger().info('Relay Deactivated: Soil is moist.')

def main(args=None):
    rclpy.init(args=args)
    node = RelayController()
    rclpy.spin(node)
    rclpy.shutdown()
