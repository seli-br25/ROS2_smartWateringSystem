import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import OutputDevice
from datetime import datetime, timedelta

class RelayController(Node):
    def __init__(self):
        super().__init__('relay_controller')
        self.subscription = self.create_subscription(
            String,
            'soil_moisture',
            self.control_relay,
            10
        )
        self.command_subscription = self.create_subscription(  # Listen to the continue_command topic
            String,
            'continue_command',
            self.handle_continue_command,
            10
        )
        self.status_publisher = self.create_publisher(String, 'watering_status', 10)
        self.relay = OutputDevice(27, active_high=True, initial_value=False)  # GPIO27
        self.pump_active = False
        self.paused = False
        self.last_watering_time = None

    def control_relay(self, msg):
        if self.paused:
            self.get_logger().info("Paused: Waiting for /refilled command.")
            return

        current_time = datetime.now()
        if msg.data == "dry":
            if not self.pump_active:
                self.activate_pump(current_time)
            elif self.pump_active and (current_time - self.last_watering_time > timedelta(seconds=5)):
                self.deactivate_pump()
                self.publish_status("Soil is still dry after watering – check the water reservoir!")
                self.paused = True
        else:
            if self.pump_active:
                self.deactivate_pump()
                self.publish_status("Plant successfully watered!")

    def activate_pump(self, current_time):
        self.relay.on()
        self.pump_active = True
        self.last_watering_time = current_time
        self.publish_status("Pump activated")
        self.get_logger().info('Pump activated: Soil is dry.')

    def deactivate_pump(self):
        self.relay.off()
        self.pump_active = False
        self.publish_status("Pump deactivated")
        self.get_logger().info('Pump deactivated.')

    def reset_paused_state(self):
        self.paused = False
        self.get_logger().info("Paused state reset: Resuming moisture checks.")
        self.publish_status("System resumed: Ready to monitor soil moisture.")

    def publish_status(self, message):
        msg = String()
        msg.data = message
        self.status_publisher.publish(msg)
        self.get_logger().info(f'Status published: {message}')

    def handle_continue_command(self, msg):
        if msg.data == "continue":
            self.reset_paused_state()

def main(args=None):
    rclpy.init(args=args)
    node = RelayController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
