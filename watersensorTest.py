import spidev
from time import sleep

# SPI initialisieren
spi = spidev.SpiDev()
spi.open(0, 0)  # SPI-Bus 0, Gerät 0 (CE0)
spi.max_speed_hz = 1350000

# Funktion zum Lesen eines ADC-Kanals
def read_adc(channel):
    assert 0 <= channel <= 7, "Channel muss zwischen 0 und 7 liegen"
    # SPI-Nachricht senden: Start-Bit, Kanalnummer, Leerdaten
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    # Debug: Zeige die rohen SPI-Daten an
    print(f"Raw SPI Response: {adc}")
    # Ergebnis aus zwei Bytes zusammenfügen
    data = ((adc[1] & 3) << 8) + adc[2]
    return data

try:
    while True:
        value = read_adc(4)  # Kanal 4 (AN-Pin) auslesen
        print(f"CH1 (Unten1-Pin) ADC-Wert: {value}")
        sleep(1)
except KeyboardInterrupt:
    # SPI-Verbindung schließen, wenn das Programm beendet wird
    spi.close()
    print("SPI-Verbindung geschlossen.")
