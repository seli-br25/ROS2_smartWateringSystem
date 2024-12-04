import spidev
from time import sleep

spi = spidev.SpiDev()
spi.open(0, 0)  # SPI bus 0, device 0 (CE0)
spi.max_speed_hz = 1350000

def read_adc(channel):
    assert 0 <= channel <= 7, "Channel must be between 0 and 7"
    adc = spi.xfer2([1, (8 + channel) << 4, 0])  # Start bit + channel selection
    data = ((adc[1] & 3) << 8) + adc[2]  # Combine high and low bytes
    return data

while True:
    for ch in range(8):  # Check all channels
        value = read_adc(ch)
        print(f"Channel {ch}: {value}")
    sleep(1)