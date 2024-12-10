import RPi.GPIO as GPIO
import time

def main():
    # Set up GPIO
    GPIO.setmode(GPIO.BCM)  # Use BCM numbering
    GPIO.setup(12, GPIO.IN)  # Set GPIO12 as input

    try:
        print("Reading from GPIO12 (AN)...")
        while True:
            value = GPIO.input(12)  # Read the digital input
            print(f"Digital Value: {'HIGH' if value else 'LOW'}")
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        GPIO.cleanup()  # Reset GPIO settings

if __name__ == "__main__":
    main()
