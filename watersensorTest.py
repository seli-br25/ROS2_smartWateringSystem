import time
from gpiozero import MCP3008

def main():
    # Configure GPIO12 (ADC4)
    adc = MCP3008(channel=4)  # Channel 4 corresponds to GPIO12 (ADC4)
    
    try:
        print("Reading water sensor values from GPIO12 (ADC4)...")
        while True:
            # Read the raw ADC value
            value = adc.value  # Returns a value between 0.0 and 1.0
            scaled_value = int(value * 1023)  # Scale to 0–1023 (10-bit ADC)

            # Print the value
            print(f"Water Sensor Raw Value: {scaled_value}")

            # Determine status
            if scaled_value < 50:
                print("Status: Dry")
            else:
                print("Status: Moist")

            time.sleep(1)  # Wait for 1 second
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        adc.close()  # Cleanup ADC connection

if __name__ == "__main__":
    main()
