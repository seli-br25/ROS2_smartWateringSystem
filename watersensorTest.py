from gpiozero import InputDevice

sensor = InputDevice(17)  # GPIO17

while True:
    if sensor.is_active:
        print("Moisture detected!")
    else:
        print("Soil is dry!")
