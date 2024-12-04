import RPi.GPIO as GPIO
from time import sleep

MOISTURE_PIN = 17  # GPIO pin for the water sensor

GPIO.setmode(GPIO.BCM)
GPIO.setup(MOISTURE_PIN, GPIO.IN)

try:
    while True:
        if GPIO.input(MOISTURE_PIN) == GPIO.HIGH:
            print("Moisture detected!")
        else:
            print("Soil is dry!")
        sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()
