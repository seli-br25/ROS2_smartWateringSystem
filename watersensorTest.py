import time
import spidev

def read_adc_channel(channel):
    # SPI setup for MCP3008
    spi = spidev.SpiDev()
    spi.open(0, 0)  # SPI bus 0, device 0
    spi.max_speed_hz = 1350000

    try:
        # Read analog data from the specified MCP3008 channel
        adc = spi.xfer2([1, (8 + channel) << 4, 0])
        value = ((adc[1] & 3) << 8) + adc[2]
        return value
    finally:
        spi.close()

def main():
    print("Reading water sensor values from GPIO12 (AN / ADC4)...")
    while True:
        # Read from MCP3008 channel 4 (corresponding to GPIO12)
        value = read_adc_channel(4)

        # Print the value and determine status
        print(f"Raw Value: {value}")
        if value < 50:
            print("Status: Dry")
        else:
            print("Status: Moist")

        time.sleep(1)

if __name__ == "__main__":
    main()
