import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import serial

RELAY_PIN = 7  # GPIO pin for the relay

class SmartWateringNode(Node):
    def __init__(self):
        super().__init__('smart_watering_node')

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RELAY_PIN, GPIO.OUT)
        GPIO.output(RELAY_PIN, GPIO.LOW)  # Pump is off by default

        # Serial connection setup
        self.serial_port = serial.Serial('/dev/ttyACM0', 57600, timeout=1)

        # ROS2 publisher
        self.moisture_publisher = self.create_publisher(String, 'moisture_status', 10)

        # Timer for reading moisture status
        self.create_timer(1.0, self.read_moisture_status)

    def read_moisture_status(self):
        """Reads moisture status from the Arduino."""
        if self.serial_port.in_waiting > 0:
            data = self.serial_port.readline().decode('utf-8').strip()
            self.get_logger().info(f"Moisture status: {data}")
            
            msg = String()
            msg.data = data
            self.moisture_publisher.publish(msg)

            # Control the pump based on the moisture status
            if data == "dry":
                GPIO.output(RELAY_PIN, GPIO.HIGH)  # Turn on the pump
            elif data == "moist":
                GPIO.output(RELAY_PIN, GPIO.LOW)  # Turn off the pump

    def destroy_node(self):
        """Clean up GPIO resources."""
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SmartWateringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
