import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import RPi.GPIO as GPIO

RELAY_PIN = 17  # GPIO-Pin für das Relais (anpassen!)

class MoistureController(Node):
    def __init__(self):
        super().__init__('moisture_controller')
        self.publisher_ = self.create_publisher(String, 'moisture_status', 10)
        self.subscription = self.create_subscription(
            String,
            'moisture_status',
            self.listener_callback,
            10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)  # Passe den Port an!
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RELAY_PIN, GPIO.OUT)
        GPIO.output(RELAY_PIN, GPIO.LOW)  # Pumpe standardmäßig aus

    def read_and_publish(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            msg = String()
            msg.data = line
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received moisture status: {msg.data}')
        if msg.data == 'dry':
            GPIO.output(RELAY_PIN, GPIO.HIGH)  # Pumpe einschalten
            self.get_logger().info('Pump ON')
        elif msg.data == 'moist':
            GPIO.output(RELAY_PIN, GPIO.LOW)  # Pumpe ausschalten
            self.get_logger().info('Pump OFF')

def main(args=None):
    rclpy.init(args=args)
    node = MoistureController()

    try:
        while rclpy.ok():
            node.read_and_publish()
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
