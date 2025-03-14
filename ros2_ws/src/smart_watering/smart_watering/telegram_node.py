import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
from datetime import datetime


class TelegramNode(Node):
    def __init__(self):
        super().__init__('telegram_node')
        self.subscription_moisture = self.create_subscription(
            String,
            'soil_moisture',
            self.moisture_callback,
            10
        )
        self.subscription_status = self.create_subscription(
            String,
            'watering_status',
            self.status_callback,
            10
        )
        self.status_publisher = self.create_publisher(String, 'continue_command', 10)
        self.telegram_bot_token = ""
        self.telegram_chat_id = ""
        self.watering_log = []
        self.last_watering_time = None
        self.current_moisture = "Unknown"
        self.wasActivated = False
        self.last_update_id = 0

    def moisture_callback(self, msg):
        self.current_moisture = msg.data

    def status_callback(self, msg):
        message = msg.data
        if "Pump activated" in message:
            self.last_watering_time = datetime.now()
            self.watering_log.append(f"{self.last_watering_time.strftime('%Y-%m-%d %H:%M:%S')}: \n {message}")
            self.wasActivated = True
        elif "Pump deactivated" in message and self.wasActivated == True:
            self.watering_log.append(f"{self.last_watering_time.strftime('%Y-%m-%d %H:%M:%S')}: \n {message}")
            self.wasActivated = False
        elif "check the water reservoir!" in message:
            if self.last_watering_time is None:
                return
            self.watering_log.append(
                f"{self.last_watering_time.strftime('%Y-%m-%d %H:%M:%S')}: \n Water pump was stopped due to possible error")
        self.send_telegram_message(message)

    def send_telegram_message(self, text):
        url = f"https://api.telegram.org/bot{self.telegram_bot_token}/sendMessage"
        payload = {"chat_id": self.telegram_chat_id, "text": text}
        for _ in range(3):  # Retry 3 times if needed
            response = requests.post(url, json=payload)
            if response.status_code == 200:
                self.get_logger().info(f"Message sent: {text}")
                return
            self.get_logger().error(f"Retrying message send: {response.text}")

    def handle_continue_command(self):
        if self.last_watering_time is None:
            self.get_logger().info("Cannot continue: No previous watering event.")
            return
        self.publish_continue_command()
        self.send_telegram_message("System resumed: Soil monitoring is active again.")
        self.watering_log.append(
            f"{self.last_watering_time.strftime('%Y-%m-%d %H:%M:%S')}: \n Water pump was activated again with the continue-command")

    def handle_status_command(self):
        watering_history = "\n".join(self.watering_log[-5:]) if self.watering_log else "No watering history available."
        last_watered = self.last_watering_time.strftime('%Y-%m-%d %H:%M:%S') if self.last_watering_time else "Never"
        message = (
            f"Plant Status:\n"
            f"- Current Moisture: {self.current_moisture}\n"
            f"- Last Watered: {last_watered}\n"
            f"- Watering History (last 5 events):\n{watering_history}"
        )
        self.send_telegram_message(message)

    def publish_continue_command(self):
        msg = String()
        msg.data = "continue"
        self.status_publisher.publish(msg)
        self.get_logger().info("Continue command sent.")

    def fetch_telegram_updates(self):
        url = f"https://api.telegram.org/bot{self.telegram_bot_token}/getUpdates"
        params = {}
        if hasattr(self, 'last_update_id'):
            # Fetch only updates with IDs greater than the last processed update
            params['offset'] = self.last_update_id + 1

        response = requests.get(url, params=params)

        if response.status_code == 200:
            updates = response.json().get('result', [])
            for update in updates:
                message = update.get('message', {}).get('text', '')
                update_id = update.get('update_id', 0)

                # Update last_update_id to the current update ID
                self.last_update_id = update_id

                if message == "/status":
                    self.handle_status_command()
                elif message == "/continue":
                    self.handle_continue_command()
        else:
            self.get_logger().error(f"Failed to fetch updates: {response.text}")


def main(args=None):
    rclpy.init(args=args)
    node = TelegramNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.fetch_telegram_updates()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
