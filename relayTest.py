from gpiozero import OutputDevice
from time import sleep

relay = OutputDevice(27, active_high=True, initial_value=False)  # GPIO27

while True:
    print("Turning ON relay...")
    relay.on()
    sleep(2)
    print("Turning OFF relay...")
    relay.off()
    sleep(2)
