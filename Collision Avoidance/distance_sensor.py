import time
import busio
import digitalio
import board
from adafruit_mcp3xxx.mcp3008 import MCP3008
from adafruit_mcp3xxx.analog_in import AnalogIn

spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.CE0)
mcp = MCP3008(spi, cs)
chan0 = AnalogIn(mcp, 0)  # if sensor is on CH0

while True:
        print("Raw ADC value:", chan0.value, "Voltage:", chan0.voltage)
        time.sleep(0.5)