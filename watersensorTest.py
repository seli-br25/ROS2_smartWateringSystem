from gpiozero import MCP3008
from time import sleep

# Initialize the MCP3008 channel 0 (connected to AN pin)
moisture_sensor = MCP3008(channel=0)

while True:
    # Read the moisture value (0.0 to 1.0)
    moisture_value = moisture_sensor.value * 100  # Convert to percentage
    print(f"Moisture Level: {moisture_value:.2f}%")
    sleep(1)
