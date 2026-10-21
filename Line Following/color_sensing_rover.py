import time
import board
import busio
import adafruit_tcs34725

THRESHOLD = 4975   # set this from your own measurements (see below)

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_tcs34725.TCS34725(i2c)
sensor.integration_time = 50
sensor.gain = 4

try:
    while True:
        r, g, b, c = sensor.color_raw
        name = "white" if c > THRESHOLD else "black"
        print(f"C={c}  {name}")
        time.sleep(0.2)
except KeyboardInterrupt:
    print("Stopped by user")