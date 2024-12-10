import spidev  # Library for SPI communication
import time

def read_adc_channel(channel, spi):
    # Perform SPI transaction and bit shifting to read from MCP3008
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    value = ((adc[1] & 3) << 8) + adc[2]
    return value

def main():
    # Initialize SPI
    spi = spidev.SpiDev()
    spi.open(0, 0)  # Use bus 0, device 0
    spi.max_speed_hz = 1350000

    print("Testing MCP3008 on alternate SPI pins (GPIO11, GPIO10, GPIO9, GPIO8)...")

    try:
        while True:
            # Read analog value from channel 0
            value = read_adc_channel(0, spi)
            print(f"Analog Value from Channel 0: {value}")
            
            # Determine status (example threshold for "dry" or "moist")
            if value < 50:
                print("Status: Dry")
            else:
                print("Status: Moist")

            time.sleep(1)  # Delay for readability
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        spi.close()  # Close SPI connection

if __name__ == "__main__":
    main()
