import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import spidev
from time import sleep

class MoisturePublisher(Node):
    def __init__(self):
        super().__init__('moisture_publisher')
        self.publisher_ = self.create_publisher(String, 'soil_moisture', 10)
        self.timer = self.create_timer(1.0, self.publish_moisture_status)
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)  # SPI Bus 0, Device 0 (CE0)
        self.spi.max_speed_hz = 1350000

    def read_adc(self, channel):
        assert 0 <= channel <= 7, "Channel must be between 0 and 7"
        adc = self.spi.xfer2([1, (8 + channel) << 4, 0])
        data = ((adc[1] & 3) << 8) + adc[2]
        return data

    def publish_moisture_status(self):
        try:
            value = self.read_adc(4)  # Read from CH4 (AN Pin)
            status = "dry" if value < 512 else "moist"  # Adjust threshold as needed
            msg = String()
            msg.data = status
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: {status} (ADC Value: {value})')
        except Exception as e:
            self.get_logger().error(f'Error reading sensor: {e}')

    def destroy_node(self):
        self.spi.close()  # Ensure SPI connection is closed
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MoisturePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
