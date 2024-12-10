import spidev
import time

def read_channel(spi, channel):
    # Perform SPI transaction and bit shifting to read from MCP3008
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data

def main():
    # Setup SPI
    spi = spidev.SpiDev()
    spi.open(0, 0)  # Open SPI bus 0, device 0
    spi.max_speed_hz = 1350000

    try:
        print("Reading water sensor values from GPIO12 (ADC4)...")
        while True:
            # Read from MCP3008 channel 4 (ADC4)
            value = read_channel(spi, 4)
            # Print the value
            print(f"Water Sensor Value: {value}")

            # Determine status
            if value < 50:
                print("Status: Dry")
            else:
                print("Status: Moist")

            time.sleep(1)  # Wait for 1 second before next reading
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        spi.close()  # Cleanup SPI connection

if __name__ == "__main__":
    main()
