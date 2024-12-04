import spidev
from time import sleep

# Open SPI bus
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0 (CE0)
spi.max_speed_hz = 1350000

def read_adc(channel):
    assert 0 <= channel <= 7, "ADC channel must be 0-7"
    adc = spi.xfer2([1, (8 + channel) << 4, 0])  # Start bit + channel selection
    data = ((adc[1] & 3) << 8) + adc[2]  # Combine high and low bytes
    return data

while True:
    value = read_adc(0)  # Read CH0
    print(f"ADC Value: {value}")  # Should be 0-1023
    sleep(1)