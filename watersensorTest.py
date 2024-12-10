from machine import Pin, ADC
import time

def main():
    # Initialize ADC on GPIO12 (AN)
    adc = ADC(Pin(12))  # Configure GPIO12 as ADC input
    adc.atten(ADC.ATTN_11DB)  # Set attenuation for input range up to ~3.3V

    try:
        print("Reading water sensor values from GPIO12 (AN)...")
        while True:
            # Read raw ADC value (usually in the range 0-4095 for 12-bit resolution)
            value = adc.read()

            # Print the value
            print(f"Water Sensor Value: {value}")

            # Determine status
            if value < 2000:  # Adjust this threshold based on your sensor
                print("Status: Dry")
            else:
                print("Status: Moist")

            time.sleep(1)  # Wait for 1 second
    except KeyboardInterrupt:
        print("Exiting...")

if __name__ == "__main__":
    main()
