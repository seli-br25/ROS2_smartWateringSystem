import spidev
import time

def read_adc_channel(channel):
    spi = spidev.SpiDev()
    spi.open(0, 0)  # SPI bus 0, device 0
    spi.max_speed_hz = 1350000

    try:
        # Send start bit, single-ended bit, and channel number
        adc = spi.xfer2([1, (8 + channel) << 4, 0])
        value = ((adc[1] & 3) << 8) + adc[2]  # Combine bits to get ADC value
        return value
    finally:
        spi.close()

def main():
    print("Reading water sensor values from GPIO12 (AN)...")
    while True:
        # Read from MCP3008 channel 4 (GPIO12 / ADC4)
        value = read_adc_channel(4)

        # Print the raw ADC value and status
        print(f"Raw Value: {value}")
        if value < 50:  # Adjust this threshold based on your sensor
            print("Status: Dry")
        else:
            print("Status: Moist")

        time.sleep(1)  # Wait for 1 second

if __name__ == "__main__":
    main()
